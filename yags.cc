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
      globalPredictorSize(params->globalPredictorSize),
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

    printf("Initilizing taken/notTaken counters with %u 1s\n",this->globalCtrBits);
    for(uint32_t count = 0; count < this->globalPredictorSize; count++)
    {
    	this->takenCounters[count].ctr.setBits(this->globalCtrBits);
    	this->takenCounters[count].tag = 0;
    	this->notTakenCounters[count].ctr.setBits(this->globalCtrBits);
    	this->notTakenCounters[count].tag = 0;
    }

    //set up the mask for indexing from branch address
    this->choicePredictorMask = this->choicePredictorSize - 1;
    this->globalPredictorMask = this->globalPredictorSize - 1;
    this->globalHistoryMask = mask(this->globalCtrBits);

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
	bool choicePred, finalPred;
	unsigned choiceCountersIdx = ((branchAddr >> instShiftAmt) & this->choicePredictorMask);
	//indexing into either takenPredictor or notTakenPredictor
   	unsigned globalPredictorIdx = ((branchAddr >> instShiftAmt) ^ this->globalHistoryReg) & this->globalPredictorMask;

   	//printf("%u,%u\n",choiceCountersIdx,globalPredictorIdx);
   	assert(choiceCountersIdx < this->choicePredictorSize);
   	assert(globalPredictorIdx < this->globalPredictorSize);

   	uint32_t tag = (branchAddr >> instShiftAmt) & this->tagsMask;
   	BPHistory *history = new BPHistory;
  	history->globalHistoryReg = this->globalHistoryReg;
   	//printf("Getting choice prediction\n");
   	choicePred = this->choiceCounters[choiceCountersIdx].read() > this->choiceThreshold;
   	if(choicePred)
   	{
   		//the choice predict taken, try to look into notTaken predictor/cache
   		//printf("Getting taken prediction\n");
   		if(tag == this->takenCounters[globalPredictorIdx].tag)
   		{
   			//printf("USING PREDICTION FROM TAKEN PREDICTOR\n");
   			finalPred = this->takenCounters[globalPredictorIdx].ctr.read() > this->globalPredictorThreshold;
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
   		if(tag == this->notTakenCounters[globalPredictorIdx].tag)
   		{
   			//printf("USING PREDICTION FROM NOT TAKEN PREDICTOR\n");
   			finalPred = this->notTakenCounters[globalPredictorIdx].ctr.read() > this->globalPredictorThreshold;
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
    				this->takenCounters[globalPredictorIdx].tag = (branchAddr >> instShiftAmt) & this->tagsMask;
    				this->takenCounters[globalPredictorIdx].ctr.increment();
    				this->choiceCounters[choiceCountersIdx].increment();

    			}
    			else if(history->finalPred == true && taken == false)
    			{
    				//update the not taken predictor(cache)
    				this->notTakenCounters[globalPredictorIdx].tag = (branchAddr >> instShiftAmt) & this->tagsMask;
    				this->notTakenCounters[globalPredictorIdx].ctr.decrement();
    				this->choiceCounters[choiceCountersIdx].decrement();
    			}
    		break;
    		case 1:
    			//the taken predictor was used, choice predictor indicates not taken
    			this->takenCounters[globalPredictorIdx].tag = (branchAddr >> instShiftAmt) & this->tagsMask;
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
    			if(taken)
    			{
    				//choicePredictor bias not taken and the result is taken
    				this->takenCounters[globalPredictorIdx].ctr.increment();
    			}
    			else
    			{
    				//choicePredictor bias not taken and the result is not taken
    				this->takenCounters[globalPredictorIdx].ctr.decrement();
    			}
    		break;
    		case 2:
    			//the not taken predictor was used
    			this->notTakenCounters[globalPredictorIdx].tag = (branchAddr >> instShiftAmt) & this->tagsMask;
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

    			if(taken)
    			{
    				//choicePredictor bias taken and the result is taken
    				this->notTakenCounters[globalPredictorIdx].ctr.increment();
    			}
    			else
    			{
    				//choicePredictor bias taken and the result is not taken
    				this->notTakenCounters[globalPredictorIdx].ctr.decrement();
    			}
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
