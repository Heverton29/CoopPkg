/**
 *  TaskState.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASKSTATE_H_
#define TASKSTATE_H_

typedef enum { 
		NOT_ASSIGNED, //0
		WAITING,      //1
		EXECUTING,    //2
		ABORTED,      //3
		FAILED,       //4
		SUCCEEDED     //5
			} TaskState;

#endif /* TASKSTATE_H_ */
