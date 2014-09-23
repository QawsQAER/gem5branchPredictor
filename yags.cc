/* @file
 * Implementation of a YAGS branch predictor
 *
 * 18-640 Foundations of Computer Architecture
 * Carnegie Mellon University
 *
 */

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "cpu/pred/yags.hh"


/*
 * Constructor for YagsBP
 */
YagsBP::YagsBP(const Params *params)
    : BPredUnit(params), instShiftAmt(params->instShiftAmt),
      globalHistoryReg(0),
      globalHistoryBits(ceilLog2(params->globalPredictorSize)),
      choicePredictorSize(params->choicePredictorSize),
      choiceCtrBits(params->choiceCtrBits),
      globalPredictorSize(params->globalPredictorSize / _SET_ASSOCITY),
      globalCtrBits(params->globalCtrBits)
{
	//judging the predictor size
    if(!isPowerOf2(this->globalPredictorSize))
    	fatal("Invalid global predictor size!\n");
    if(!isPowerOf2(this->choicePredictorSize))
    	fatal("Invalid choice predictor size!\n");

    //set up the tables of counters and Tags
    //taken and notTaken predictor should share the same size as globalPredictorSize
    this->choiceCounters.resize(this->choicePredictorSize);
    this->takenCounters.resize(this->globalPredictorSize);
    this->notTakenCounters.resize(this->globalPredictorSize);

    //initilize the counter's values
    printf("Initilizing choiceCounters with %u 1s\n",this->choiceCtrBits);
    for(uint32_t count = 0; count < this->choicePredictorSize; count++)
    {
    	this->choiceCounters[count].setBits(this->choiceCtrBits);
    }

    this->initCache();
    //set up the mask for indexing from branch address
    this->choicePredictorMask = this->choicePredictorSize - 1;
    this->globalPredictorMask = this->globalPredictorSize - 1;
    this->globalHistoryMask = mask(this->globalHistoryBits);
    this->globalHistoryUnusedMask = this->globalHistoryMask - (this->globalHistoryMask >> (ceilLog2(_SET_ASSOCITY)));
    printf("globalHistoryBits is %u\n",this->globalHistoryBits);
    printf("globalHistoryMask is %08x\n",this->globalHistoryMask);
    printf("globalPredictorMask is %08x\n",this->globalPredictorMask);
    printf("globalHistoryUnusedMask is %08x\n",this->globalHistoryUnusedMask);
    //set up the threshold for branch prediction
    this->choiceThreshold = (ULL(1) << (this->choiceCtrBits - 1)) - 1;;
    this->globalPredictorThreshold = (ULL(1) << (this->globalCtrBits - 1)) - 1;

    //using 8 bits of address as tags.
    this->tagsMask = mask(_YAGS_TAG_LENGTH);
    printf("YagsBP() Constructor done\n");
}

/*
 * Actions for an unconditional branch
 */
void
YagsBP::uncondBranch(void * &bpHistory)
{
    BPHistory *history = new BPHistory;
    history->globalHistoryReg = this->globalHistoryReg;
    history->takenUsed = 0;
    history->notTakenPred = true;
    history->takenPred = true;
    history->finalPred = true;
    bpHistory = static_cast<void*>(history);
    updateGlobalHistReg(true);
}

/*
 * Actions for squash
 */
void
YagsBP::squash(void *bpHistory)
{
	if(bpHistory)
    {
    	BPHistory *history = static_cast<BPHistory*>(bpHistory);
    	this->globalHistoryReg = history->globalHistoryReg;
    	delete history;
    }
}

/*
 * Lookup the actual branch prediction.
 */
bool
YagsBP::lookup(Addr branchAddr, void * &bpHistory)
{
	//printf("Performing lookup\n");
	bool choicePred, finalPred = true;
	unsigned choiceCountersIdx = ((branchAddr >> instShiftAmt) & this->choicePredictorMask);
	//indexing into either takenPredictor or notTakenPredictor
   	unsigned globalPredictorIdx = ((branchAddr >> instShiftAmt) ^ this->globalHistoryReg) & this->globalPredictorMask;

   	//printf("%u,%u\n",choiceCountersIdx,globalPredictorIdx);
   	assert(choiceCountersIdx < this->choicePredictorSize);
   	assert(globalPredictorIdx < this->globalPredictorSize);

   	uint32_t tag = ((branchAddr >> instShiftAmt) & this->tagsMask) | ((this->globalHistoryReg & this->globalHistoryUnusedMask) << (ceilLog2(_SET_ASSOCITY)));
   	BPHistory *history = new BPHistory;
  	history->globalHistoryReg = this->globalHistoryReg;
   	//printf("Getting choice prediction\n");
   	choicePred = this->choiceCounters[choiceCountersIdx].read() > this->choiceThreshold;
   	if(choicePred)
   	{
   		//the choice predict taken, try to look into notTaken predictor/cache
   		//printf("Getting taken prediction\n");
   		if(lookupTakenCache(globalPredictorIdx,tag,&finalPred))
   		{
   			//printf("USING PREDICTION FROM TAKEN PREDICTOR\n");
   			history->takenPred = finalPred;
   			history->takenUsed = 1;
   		}
   		else
   		{
   			history->takenUsed = 0;
   			finalPred = choicePred;
   		}
   	}
   	else
   	{
   		//the choice predict not taken, try to look into Taken predictor/cache
   		//printf("Getting not taken prediction\n");
   		if(lookupNotTakenCache(globalPredictorIdx,tag,&finalPred))
   		{
   			//printf("USING PREDICTION FROM NOT TAKEN PREDICTOR\n");
   			history->notTakenPred = finalPred;
   			history->takenUsed = 2;
   		}
   		else
   		{
   			history->takenUsed = 0;
   			finalPred = choicePred;
   		}
   	}
   	//printf("Updating global history\n");
   	history->finalPred = finalPred;
   	bpHistory = static_cast<void*>(history);
   	updateGlobalHistReg(finalPred);
    return finalPred;
}

/*
 * BTB Update actions
 */
void
YagsBP::btbUpdate(Addr branchAddr, void * &bpHistory)
{
    this->globalHistoryReg &= (globalHistoryMask & ~ULL(1));
}

/*
 * Update data structures after getting actual decison 
 */
void
YagsBP::update(Addr branchAddr, bool taken, void *bpHistory, bool squashed)
{
	//printf("Performing update\n");
    if(bpHistory)
    {
    	BPHistory *history = static_cast<BPHistory *>(bpHistory);
    	unsigned choiceCountersIdx = ((branchAddr >> instShiftAmt) & this->choicePredictorMask);
    	//indexing into either takenPredictor or notTakenPredictor
    	unsigned globalPredictorIdx = ((branchAddr >> instShiftAmt) ^ history->globalHistoryReg) & this->globalPredictorMask;
   		uint32_t tag = ((branchAddr >> instShiftAmt) & this->tagsMask) | ((history->globalHistoryReg & this->globalHistoryUnusedMask) << (ceilLog2(_SET_ASSOCITY)));
      assert(choiceCountersIdx < this->choicePredictorSize);
   		assert(globalPredictorIdx < this->globalPredictorSize);
    	switch(history->takenUsed)
    	{
    		case 0:
    			//the choice predictor was used
    			if(history->finalPred == taken)
    			{
    				//the case that the prediction is correct
    				if(taken == true)
    					this->choiceCounters[choiceCountersIdx].increment();
    				else
    					this->choiceCounters[choiceCountersIdx].decrement();

    			}
    			else if(history->finalPred == false && taken == true)
    			{
    				//update the taken predictor(cache)
            this->updateTakenCache(globalPredictorIdx,tag,taken);
    				this->choiceCounters[choiceCountersIdx].increment();

    			}
    			else if(history->finalPred == true && taken == false)
    			{
    				//update the not taken predictor(cache)
            this->updateNotTakenCache(globalPredictorIdx,tag,taken);
    				this->choiceCounters[choiceCountersIdx].decrement();
    			}
    		break;
    		case 1:
    			//the taken predictor was used, choice predictor indicates not taken
    			if(taken == history->takenPred && (!taken) == false)
    			{
    				
    			}
    			else
    			{
    				if(taken)
    					this->choiceCounters[choiceCountersIdx].increment();
    				else
    					this->choiceCounters[choiceCountersIdx].decrement();
    			}
          this->updateTakenCache(globalPredictorIdx,tag,taken);
    		break;
    		case 2:
    			//the not taken predictor was used
    			if(taken == history->notTakenPred && (!taken) == true)
    			{
    				
    			}
    			else
    			{
    				if(taken)
    					this->choiceCounters[choiceCountersIdx].increment();
    				else
    					this->choiceCounters[choiceCountersIdx].decrement();
    			}
          this->updateNotTakenCache(globalPredictorIdx,tag,taken);
    		break;
    	}

    	if(squashed)
    	{
    		if(taken)
    			this->globalHistoryReg = (history->globalHistoryReg << 1) | 1;
    		else
    			this->globalHistoryReg = (history->globalHistoryReg << 1);
    		this->globalHistoryReg &= this->globalHistoryMask;
    	}
    	else
    		delete history;
    }

}

/*
 * Retire Squashed Instruction
 */
void
YagsBP::retireSquashed(void *bp_history)
{
	if(bp_history)
    {
    	BPHistory *history = static_cast<BPHistory*>(bp_history);
    	delete history;
    }
}

/*
 * Global History Registor Update 
 */
void
YagsBP::updateGlobalHistReg(bool taken)
{
    this->globalHistoryReg = taken ? this->globalHistoryReg << 1 | 1 : this->globalHistoryReg << 1;
    this->globalHistoryReg = this->globalHistoryReg & this->globalHistoryMask;
}

bool YagsBP::lookupTakenCache(const unsigned idx,const uint32_t tag, bool *taken)
{
  bool found = 0;
  for(uint8_t count = 0;count < _SET_ASSOCITY;count++)
  {
    if(this->takenCounters[idx].tag[count] == tag)
    {
      this->updateTakenCacheLRU(idx,count);  
      *taken = this->takenCounters[idx].ctr[count].read() > this->globalPredictorThreshold;
      found = true;
      return true;
    }
  }

  if(found)
    return true;
  else
    return false;
}

bool YagsBP::lookupNotTakenCache(const unsigned idx,const uint32_t tag,bool *taken)
{

  bool found = 0;
  for(uint8_t count = 0;count < _SET_ASSOCITY;count++)
  {
    if(this->notTakenCounters[idx].tag[count] == tag)
    {
      this->updateNotTakenCacheLRU(idx,count);
      *taken = this->notTakenCounters[idx].ctr[count].read() > this->globalPredictorThreshold;
      found = true;
      return true;
    }
  }

  if(found)
    return true;
  else
    return false;
}

void YagsBP::updateTakenCache(const unsigned idx, const uint32_t tag,const bool taken)
{
  bool found = false;
  for(uint8_t count = 0;count < _SET_ASSOCITY;count++)
  {
    if(this->takenCounters[idx].tag[count] == tag)
    {
      this->updateTakenCacheLRU(idx,count);
      if(taken)
        this->takenCounters[idx].ctr[count].increment();
      else
        this->takenCounters[idx].ctr[count].decrement();
      found = true;
    }
  }
  //if did not find any matching tag
  //replace the least-recently-used one
  
  if(!found)
  {
    uint8_t LRU = this->takenCounters[idx].LRU;
    this->takenCounters[idx].tag[LRU] = tag;
    //reset the counter
    this->takenCounters[idx].ctr[LRU].setBits(this->globalCtrBits);
    
    if(taken)
      this->takenCounters[idx].ctr[LRU].increment();
    else
      this->takenCounters[idx].ctr[LRU].decrement();
    
  }
}

void YagsBP::updateNotTakenCache(const unsigned idx, const uint32_t tag,const bool taken)
{
  bool found = false;
  for(uint8_t count = 0;count < _SET_ASSOCITY;count++)
  {
    if(this->notTakenCounters[idx].tag[count] == tag)
    {
      this->updateNotTakenCacheLRU(idx,count);
      if(taken)
        this->notTakenCounters[idx].ctr[count].increment();
      else
        this->notTakenCounters[idx].ctr[count].decrement();

      found = true;
    }
  }
  //if did not find any matching tag
  //replace the least-recently-used one
  if(!found)
  {
    uint8_t LRU = this->takenCounters[idx].LRU;
    this->notTakenCounters[idx].tag[LRU] = tag;
    //reset the counter 
    this->notTakenCounters[idx].ctr[LRU].setBits(this->globalCtrBits);
    if(taken)
      this->notTakenCounters[idx].ctr[LRU].increment();
    else
      this->notTakenCounters[idx].ctr[LRU].decrement();
  }
}

void YagsBP::initCache()
{
    printf("Initilizing taken/notTaken counters with %u 1s\n",this->globalCtrBits);
    for(uint32_t count = 0; count < this->globalPredictorSize; count++)
    {
      for(uint8_t count_entry = 0;count_entry < _SET_ASSOCITY;count_entry++)
      {
        this->takenCounters[count].ctr[count_entry].setBits(this->globalCtrBits);
        this->takenCounters[count].tag[count_entry] = 0;
        this->takenCounters[count].used[count_entry] = count_entry;
        this->notTakenCounters[count].ctr[count_entry].setBits(this->globalCtrBits);
        this->notTakenCounters[count].tag[count_entry] = 0;
        this->notTakenCounters[count].used[count_entry] = count_entry;
      }
      this->takenCounters[count].LRU = 0;
      this->notTakenCounters[count].LRU = 0;
    }
    printf("Cache initilization done\n");
}

void YagsBP::updateTakenCacheLRU(const unsigned idx, const uint8_t entry_idx)
{
  uint8_t threshold_used = this->takenCounters[idx].used[entry_idx];
  this->takenCounters[idx].used[entry_idx] = _SET_ASSOCITY - 1;
  for(uint8_t count = 0; count < _SET_ASSOCITY;count++)
  {
    if(this->takenCounters[idx].used[count] > threshold_used && count != entry_idx)
      this->takenCounters[idx].used[count]--;
    if(this->takenCounters[idx].used[count] == 0)
      this->takenCounters[idx].LRU = count;
  }
}

void YagsBP::updateNotTakenCacheLRU(const unsigned idx, const uint8_t entry_idx)
{
  uint8_t threshold_used = this->notTakenCounters[idx].used[entry_idx];
  this->notTakenCounters[idx].used[entry_idx] = _SET_ASSOCITY - 1;
  for(uint8_t count = 0; count < _SET_ASSOCITY;count++)
  {
    if(this->notTakenCounters[idx].used[count] > threshold_used && count != entry_idx)
      this->notTakenCounters[idx].used[count]--;
    if(this->notTakenCounters[idx].used[count] == 0)
      this->notTakenCounters[idx].LRU = count;
  }
}