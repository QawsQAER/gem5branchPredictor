/* @file
 * Header file for YAGS branch predictor
 * 
 * 18-640 Foundations of Computer Architecture
 * Carnegie Mellon University
 *
 */

#ifndef __CPU_PRED_YAGS_PRED_HH__
#define __CPU_PRED_YAGS_PRED_HH__

#include "cpu/pred/bpred_unit.hh"
#include "cpu/pred/sat_counter.hh"

/*
 * Feel free to make any modifications, this is a skeleton code
 * to get you started.
 * Note: Do not change name of class
 */

 #define _YAGS_TAG_LENGTH 8
 #define _SET_ASSOCITY 1

class YagsBP : public BPredUnit
{
  public:
    YagsBP(const Params *params);
    void uncondBranch(void * &bp_history);
    void squash(void *bp_history);
    bool lookup(Addr branch_addr, void * &bp_history);
    void btbUpdate(Addr branch_addr, void * &bp_history);
    void update(Addr branch_addr, bool taken, void *bp_history, bool squashed);
    void retireSquashed(void *bp_history);

  private:
    void updateGlobalHistReg(bool taken);

    //init cache
    void initCache();

    //return true means hit, false means miss
    bool lookupTakenCache(const unsigned idx,const uint32_t tag, bool *taken);
    bool lookupNotTakenCache(const unsigned idx,const uint32_t tag, bool *taken);
    

    void updateTakenCache(const unsigned idx, const uint32_t tag,const bool taken);
    void updateNotTakenCache(const unsigned idx, const uint32_t tag,const bool taken);

    void updateTakenCacheLRU(const unsigned idx, const uint8_t entry_idx);
    void updateNotTakenCacheLRU(const unsigned idx, const uint8_t entry_idx);

    struct BPHistory {
        unsigned globalHistoryReg;
        // was the taken array's prediction used?
        // 0: choice Predictor used
        // 1: takenPred used
        // 2: notPred used
        uint8_t takenUsed;
        // prediction of the taken array
        // true: predict taken
        // false: predict not-taken
        bool takenPred;
        // prediction of the not-taken array
        // true: predict taken
        // false: predict not-taken
        bool notTakenPred;
        // the final taken/not-taken prediction
        // true: predict taken
        // false: predict not-taken
        bool finalPred;
    };

    struct CacheEntry
    {
        SatCounter ctr[_SET_ASSOCITY];
        uint32_t tag[_SET_ASSOCITY];
        uint8_t LRU;
        uint8_t used[_SET_ASSOCITY];
    };

    // choice predictors
    std::vector<SatCounter> choiceCounters;
    // taken direction predictors
    std::vector<CacheEntry> takenCounters;
    // not-taken direction predictors
    std::vector<CacheEntry> notTakenCounters;

    unsigned instShiftAmt;

    unsigned globalHistoryReg;
    unsigned globalHistoryBits;
    unsigned globalHistoryMask;

    unsigned choicePredictorSize;
    unsigned choiceCtrBits;
    unsigned choicePredictorMask;

    unsigned globalPredictorSize;
    unsigned globalCtrBits;
    unsigned globalPredictorMask;

    unsigned choiceThreshold;
    unsigned globalPredictorThreshold;

    unsigned tagsMask;
};

#endif // __CPU_PRED_YAGS_PRED_HH__
