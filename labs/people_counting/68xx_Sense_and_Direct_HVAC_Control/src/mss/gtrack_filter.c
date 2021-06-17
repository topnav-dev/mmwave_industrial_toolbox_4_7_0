/*
 * gtrack_filter.c
 *
 *  Created on: Jul 3, 2019
 *      Author: a0232274
 */
#include <math.h>
#include "gtrack_filter.h"
#include "mss_mmw.h"

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
extern MmwDemo_MCB    gMmwMssMCB;

float calculateUniqueness(uint32_t tid, uint8_t* indexes, uint8_t* unique, uint16_t numPoints) {
    uint16_t i = 0;
    uint16_t totalPoints = 0;
    uint16_t uniquePoints = 0;
    //iterate through each point to determine which ones are associated with the tracked object
    for (i = 0; i < numPoints; i++) {
        if (indexes[i] == tid) {
            totalPoints += 1;
            //check if point is unique
            if (unique[i>>3]&(1<<(i&0x0007))){
                uniquePoints += 1;
            }

        }
    }
    return uniquePoints/totalPoints;
}

void calculateDisplacement(uint32_t tid, trackHist * trackList) {
    float dx, dy, displacement;
    //get difference in x, y coordinates then pythagorean theorem
    dx = trackList[tid].initX - trackList[tid].target->posX;
    dy = trackList[tid].initY - trackList[tid].target->posY;
    displacement = sqrt(pow(dx, 2) + pow(dy, 2));
    if(displacement > trackList[tid].maxNetDisplacement) {
        trackList[tid].maxNetDisplacement = displacement;
    }
}

void checkTrackSwap(uint32_t tid, trackHist * trackList, MmwDemo_output_message_target * target, uint16_t numTracks) {
    uint16_t i;
    float dist;
    uint8_t locked = 0;
    uint32_t otherTID;
    for (i=0;i<numTracks;i++){
        otherTID = target[i].tid;
        //check that other is a different target and that other is a different class
        if (tid != otherTID && trackList[tid].finalDecision != trackList[otherTID].finalDecision) {
            dist = sqrt(pow(trackList[tid].target->posX - target[i].posX, 2) + pow(trackList[tid].target->posY - target[i].posY, 2));
            //System_printf("Dist: %f\n", dist);
            //we have low uniqueness and are close to a fan - first frame occurring
            if (dist < gMmwMssMCB.ifDistance && trackList[tid].locked == 0) {
                //set new init location in between the two current colliding tracks
                trackList[tid].initX = (trackList[tid].target->posX + target[i].posX)/2;
                trackList[tid].initY = (trackList[tid].target->posY + target[i].posY)/2;
                trackList[tid].locked = 1;
                trackList[tid].maxNetDisplacement = 0;
                locked = 1;
                System_printf("Target swap occurring with TID: %d\n", tid);
                break;
            } else if (dist < gMmwMssMCB.ifDistance) {
                locked = 1;
                break;
            }
        }
    }
    if (locked == 0) {
        trackList[tid].locked = 0;
    }
    return;
}

uint16_t classifierLowPassFilter(uint32_t tid, MmwDemo_output_message_classifierResults * classOut, trackHist * trackList, uint16_t numTracks) {
    uint16_t i;
    int32_t decision;
    //get classifier decision
    for (i = 0; i < numTracks; i++) {
        if (classOut[i].activeTargetID == tid) {
            decision = classOut[i].targetTag;
            break;
        }
    }
    if (decision == 1) {
        trackList[tid].classHumanCount += 1;
        trackList[tid].classNHCount -= 1;
    } else if (decision == -1) {
        trackList[tid].classHumanCount -= 1;
        trackList[tid].classNHCount += 1;
    }
    if (trackList[tid].classHumanCount > 30) {
        trackList[tid].classHumanCount = 30;
    }
    if (trackList[tid].classHumanCount < 0) {
        trackList[tid].classHumanCount = 0;
    }
    if (trackList[tid].classNHCount > 30) {
        trackList[tid].classNHCount = 30;
    }
    if (trackList[tid].classNHCount < 0){
        trackList[tid].classNHCount = 0;
    }
    //this gives classifier decision index
    return i;

}

void updateClassifierOutput(MmwDemo_output_message_classifierResults * classOut, MmwDemo_output_message_target * targets, uint8_t * indexes, uint8_t * unique, trackHist * trackList, uint16_t numPoints, uint16_t numTracks){
    uint16_t i;
    uint32_t tid;
    uint16_t classPos;
    float uniqueness;
    uint8_t activeTracks[20] = {0};
    //iterate through each track
    for (i=0; i < numTracks; i++) {
        tid = targets[i].tid;
        activeTracks[tid] = 1;
        if (trackList[tid].tid == -1) {
            //initialize trackList data
            trackList[tid].tid = tid;
            trackList[tid].initX = targets[i].posX;
            trackList[tid].initY = targets[i].posY;
            trackList[tid].maxNetDisplacement = 0;
            trackList[tid].classHumanCount = 30;
            trackList[tid].classNHCount = 0;
            trackList[tid].locked = 0;
            trackList[tid].finalDecision = -1;
        }
        //set new target so we can have up to date location info
        trackList[tid].target = &targets[i];
        //update max displacement
        calculateDisplacement(tid, trackList);
        //get uniqueness
        uniqueness = calculateUniqueness(tid, indexes, unique, numPoints);
        if (uniqueness < gMmwMssMCB.ifUniqueness){
            //System_printf("Low Uniqueness from TID: %d\n", tid);
            checkTrackSwap(tid, trackList, targets, numTracks);
        }
        //update low pass for classifier
        classPos = classifierLowPassFilter(tid, classOut, trackList, numTracks);
        //update decision based on above outputs, we do not update if locked
        if (trackList[tid].locked == 0){
            if (trackList[tid].classHumanCount > trackList[tid].classNHCount) {
                trackList[tid].finalDecision = 1;
            } else {
                trackList[tid].finalDecision = -1;
            }
            if (trackList[tid].maxNetDisplacement > gMmwMssMCB.humanDisplacement) {
                System_printf("Track ID %i set as human - displacement", tid);
                trackList[tid].finalDecision = 1;
            }
        }
        classOut[classPos].targetTag = trackList[tid].finalDecision;

    }
    //reset unused tracks
    for(i=0;i<20;i++) {
        if (activeTracks[i] == 0) {
            trackList[i].tid = -1;
        }
    }
}

int32_t MmwDemo_FilterCfg(int32_t argc, char* argv[]) {
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Set the Filter parameters */
    gMmwMssMCB.humanDisplacement = atof(argv[1]);
    gMmwMssMCB.ifUniqueness = atof(argv[2]);
    gMmwMssMCB.ifDistance = atof(argv[3]);

    return 0;
}




