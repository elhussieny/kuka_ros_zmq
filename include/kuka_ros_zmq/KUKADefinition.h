/*
 * KUKADefinition.h
 *
 *  Created on: Apr 9, 2016
 *      Author: haitham
 */

#ifndef KUKADEFINITION_H_
#define KUKADEFINITION_H_
#define PI 3.1415926
#define JOINTSNO 7
#define JOINTSControl 1
#define CartPTPControl 2
#define CartSSControl 3
#define CartDSControl 4

#define BOUND(x, min_x, max_x) (((x)<(min_x))?(min_x):((x)>(max_x))?(max_x):(x))

#define KUKA_X_MAX 0.8
#define KUKA_X_MIN 0.3
#define KUKA_Y_MAX 0.05
#define KUKA_Y_MIN -0.7
#define KUKA_Z_MAX 0.5
#define KUKA_Z_MIN -0.150



#endif /* KUKADEFINITION_H_ */
