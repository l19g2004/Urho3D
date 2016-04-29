//
// Copyright (c) 2008-2015 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once

#include <Urho3D/Engine/Application.h>

namespace Urho3D
{
    
    class Node;
    class Scene;
    
}

// All Urho3D classes reside in namespace Urho3D
using namespace Urho3D;

class RacingGame : public Application
{
    URHO3D_OBJECT(RacingGame, Application);
    
public:
    /// Construct.
    RacingGame(Context* context);
    
    /// Setup after engine initialization and before running the main loop.
    virtual void Start();
    /// Setup before engine initialization. Modifies the engine parameters.
    virtual void Setup();
    /// Cleanup after the main loop. Called by Application.
    virtual void Stop();
    float yaw_;
    float pitch_;
    int currrentViewport;
    
protected:
    /// Scene.
    SharedPtr<Scene> scene_;
    /// Camera scene node.
    SharedPtr<Node> cameraCarNode_;
    /// Camera scene node.
    SharedPtr<Node> cameraFreeNode_;
    
private:
    
    /// Create static scene content.
    void CreateScene();
    /// Create the vehicle.
    void CreateVehicle();
    /// Construct an instruction text to the UI.
    void CreateInstructions();
    /// Subscribe to necessary events.
    void SubscribeToEvents();
    
    void MoveCamera(float timeStep);
    
    /// Handle application update. Set controls to vehicle.
    void HandleUpdate(StringHash eventType, VariantMap& eventData);
    /// Handle application post-update. Update camera position after vehicle has moved.
    void HandlePostUpdate(StringHash eventType, VariantMap& eventData);
    
    /// Handle key down event to process key controls common to all samples.
    void HandleKeyDown(StringHash eventType, VariantMap& eventData);
    
    void HandleSceneUpdate(StringHash eventType, VariantMap& eventData);
    
    void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData);
    
    /// The controllable vehicle component.
    WeakPtr<Vehicle> vehicle_;
    
    /// Flag for drawing debug geometry.
    bool drawDebug_;
};
