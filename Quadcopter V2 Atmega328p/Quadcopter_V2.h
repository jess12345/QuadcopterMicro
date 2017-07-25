/*
 * Quadcopter_V2.h
 *
 * Created: 4/03/2017 8:17:40 AM
 * Author: Jessica Hu
 */ 


#ifndef QUADCOPTER_V2_H_
#define QUADCOPTER_V2_H_



// ===================== State Machine ==================== //

enum QState {CALIBRATION, IDLE, BLASTOFF, FLY, TOUCHDOWN, TEST};
void runStateMachine(void);


// ======================= Test Code ====================== //

QState runTestState(void);
QState runCalibrationState(void);
QState runIdleState(void);
QState runBlastoffState(void);
QState runFlyState(void);
QState runTouchdownState(void);


#endif /* QUADCOPTER_V2_H_ */