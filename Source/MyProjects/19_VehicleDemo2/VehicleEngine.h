//
//  VehicleEngine.h
//  Urho3D
//
//  Created by Lukas on 01.03.16.
//
//
#pragma once

#ifndef __Urho3D__VehicleEngine__
#define __Urho3D__VehicleEngine__

#include <stdio.h>

#endif /* defined(__Urho3D__VehicleEngine__) */



//=============================================================================
//=============================================================================
//const int CTRL_FORWARD = 1;


//=============================================================================
// Vehicle component, responsible for physical movement according to controls.
//=============================================================================
class VehicleEngine
{
    
public:
    /// Construct.
    VehicleEngine();
    ~VehicleEngine();
    
    // get gear
    float getGear();
    
    // get rpm
    float getRPM();
    
private:
    int rpm;
    int vehicleGear;
    
    
};

