/*
 *  NativeWrapper.mm
 *  glTest
 *
 *  Created by Interlink on 10/09/27.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "NativeWrapper.h"

void NativeWrapper::nglNormal3f(float nx, float ny, float nz){
	glNormal3f( nx, ny, nz);
}