
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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Terrain.h>
#include <Urho3D/Graphics/Skybox.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/IO/Log.h>

#include "Vehicle.h"
#include "RacingGame.h"

#include "ProcSky/ProcSky.h"

#include "Urho3D/IO/Log.h"

#include <Urho3D/DebugNew.h>

const float CAMERA_DISTANCE = 10.0f; // distance to vehicle

URHO3D_DEFINE_APPLICATION_MAIN(RacingGame)


RacingGame::RacingGame(Context* context) :
Application(context), currrentViewport(1), drawDebug_(true)
{
    // Register factory and attributes for the Vehicle component so it can be created via CreateComponent, and loaded / saved
    Vehicle::RegisterObject(context);
    ProcSky::RegisterObject(context);
}

void RacingGame::Setup()
{
    
    // Modify engine startup parameters
    engineParameters_["WindowTitle"] = GetTypeName();
    engineParameters_["LogName"]     = GetSubsystem<FileSystem>()->GetAppPreferencesDir("urho3d", "logs") + GetTypeName() + ".log";
    engineParameters_["Headless"]    = false;
    engineParameters_["Sound"]       = true;
    engineParameters_["FullScreen"]  = false;
    
    // Construct a search path to find the resource prefix with two entries:
    // The first entry is an empty path which will be substituted with program/bin directory -- this entry is for binary when it is still in build tree
    // The second and third entries are possible relative paths from the installed program/bin directory to the asset directory -- these entries are for binary when it is in the Urho3D SDK installation location
    if (!engineParameters_.Contains("ResourcePrefixPaths"))
        engineParameters_["ResourcePrefixPaths"] = ";../share/Resources;../share/Urho3D/Resources";
    

}

void RacingGame::Start()
{
    // Set custom window Title & Icon
    GetSubsystem<Graphics>()->SetWindowTitle("RacingGame");
    
    // Create static scene content
    CreateScene();
    
    // Create the controllable vehicle
    CreateVehicle();
    
    // Create the UI content
    CreateInstructions();
    
    // Subscribe to necessary events
    SubscribeToEvents();
}

void RacingGame::Stop()
{
    engine_->DumpResources(true);
}

void RacingGame::CreateScene()
{
    
    scene_ = new Scene(context_);
    
    // Create scene subsystem components
    scene_->CreateComponent<Octree>();
    scene_->CreateComponent<PhysicsWorld>();
    scene_->CreateComponent<DebugRenderer>();
    
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    
    
    // Create camera and define viewport. We will be doing load / save, so it's convenient to create the camera outside the scene,
    // so that it won't be destroyed and recreated, and we don't have to redefine the viewport on load
    cameraCarNode_ = new Node(context_);
    Camera* cameraCar = cameraCarNode_->CreateComponent<Camera>();
    cameraCar->SetFarClip(500.0f);
    cameraFreeNode_ = new Node(context_); // add a free camera to move independent from the vehicle
    Camera* cameraFree = cameraFreeNode_->CreateComponent<Camera>();
    cameraFree->SetFarClip(500.0f);
    cameraFreeNode_->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
    
    switch (currrentViewport) {
        case 0:
            GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, cameraCar));
            break;
        case 1:
            GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, cameraFree));
            break;
        default:
            break;
    }

    
    
    

    SharedPtr<RenderPath> effectRenderPath = GetSubsystem<Renderer>()->GetViewport(0)->GetRenderPath()->Clone();
    //effectRenderPath->Append(cache->GetResource<XMLFile>("PostProcess/AutoExposure.xml"));
  //  effectRenderPath->Append(cache->GetResource<XMLFile>("PostProcess/Blur.xml"));
   // effectRenderPath->SetShaderParameter("BlurRadius", Variant(0.002f) );
   // effectRenderPath->SetShaderParameter("BlurSigma", Variant(0.001f) );
   // effectRenderPath->SetEnabled("Blur", false);


    effectRenderPath->Append(cache->GetResource<XMLFile>("PostProcess/LensFlare.xml"));
    effectRenderPath->SetShaderParameter("LensFlareScale", Variant(0.05f) );
 //   effectRenderPath->SetShaderParameter("LensFlareBias", Variant(-1.0f) );
 //   effectRenderPath->SetEnabled("LensFlare", true);
 //   GetSubsystem<Renderer>()->GetViewport(0)->SetRenderPath(effectRenderPath);
    
   /*
    effectRenderPath->Append(cache->GetResource<XMLFile>("PostProcess/LensFlare.xml"));
    effectRenderPath->SetShaderParameter("LensFlareScale", Variant(0.05f) );
    effectRenderPath->SetShaderParameter("LensFlareBias", Variant(-1.0f) );
    effectRenderPath->SetEnabled("LensFlare", true);
*/
    
    // Create static scene content. First create a zone for ambient lighting and fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    Zone* zone = zoneNode->CreateComponent<Zone>();
    //zone->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
    zone->SetFogColor(Color(0.2f, 0.2f, 0.3f));
    zone->SetFogStart(300.0f);
    zone->SetFogEnd(500.0f);
    zone->SetBoundingBox(BoundingBox(-2000.0f, 2000.0f));

 /*
    // Create a directional light with cascaded shadow mapping
    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.3f, -0.5f, 0.425f));
    Light* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetCastShadows(true);
    light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
    light->SetShadowCascade(CascadeParameters(20.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    light->SetSpecularIntensity(0.5f);
*/
    
    
    
    
    // Create heightmap terrain with collision
    Node* terrainNode = scene_->CreateChild("Terrain");
    terrainNode->SetPosition(Vector3::ZERO);
    Terrain* terrain = terrainNode->CreateComponent<Terrain>();
    terrain->SetPatchSize(64);
    terrain->SetSpacing(Vector3(2.0f, 0.1f, 2.0f)); // Spacing between vertices and vertical resolution of the height map
    terrain->SetSmoothing(true);
    terrain->SetHeightMap(cache->GetResource<Image>("Textures/HeightMap.png"));
    terrain->SetMaterial(cache->GetResource<Material>("Materials/Terrain.xml"));
    // The terrain consists of large triangles, which fits well for occlusion rendering, as a hill can occlude all
    // terrain patches and other objects behind it
    terrain->SetOccluder(true);
    
    RigidBody* body = terrainNode->CreateComponent<RigidBody>();
    body->SetCollisionLayer(2); // Use layer bitmask 2 for static geometry
    body->SetFriction(0.75f);
    CollisionShape* shape = terrainNode->CreateComponent<CollisionShape>();
    shape->SetTerrain();
    
    
    // Create skybox. The Skybox component is used like StaticModel, but it will be always located at the camera, giving the
    // illusion of the box planes being far away. Use just the ordinary Box model and a suitable material, whose shader will
    // generate the necessary 3D texture coordinates for cube mapping

/*
    Node* skyNode2 = scene_->CreateChild("Sky");
    skyNode2->SetScale(80.0f); // The scale actually does not matter
    Skybox* skybox2 = skyNode2->CreateComponent<Skybox>();
    skybox2->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    skybox2->SetMaterial(cache->GetResource<Material>("Materials/Skybox.xml"));
   */
    
    
    Node* skyNode = scene_->CreateChild("ProcSkyNode");
    skyNode->SetEnabled(true);
    skyNode->SetName("ProcSkyNode");
    skyNode->SetPosition(Urho3D::Vector3(0.0, 0.0, 0.0));
    skyNode->SetRotation(Urho3D::Quaternion(1, 0, 0, 0));
    skyNode->SetScale(Urho3D::Vector3(100.0, 100.0, 100.0));
    
    
    ProcSky* procSky = skyNode->CreateComponent<ProcSky>();
    procSky->SetEnabled(true);
    
    Node* skyLightNode = skyNode->CreateChild("ProcSkyLight");
    skyLightNode->SetEnabled(true);
    skyLightNode->SetPosition(Urho3D::Vector3(0.0, 0.0, 0.0));
    
    //skyLightNode->SetRotation(Urho3D::Quaternion(0.707107, 0, -0.707107, 0));
    skyLightNode->SetRotation(Quaternion(0, 0, 0));
    skyLightNode->Yaw(-270.0);
    skyLightNode->Pitch(90-45);
    skyLightNode->Roll(0);
    
    
    skyLightNode->SetScale(Urho3D::Vector3(1, 1, 1));
    Light* skyLight = skyLightNode->CreateComponent<Light>();
    skyLight->SetLightType(LIGHT_DIRECTIONAL);
    skyLight->SetColor(Urho3D::Color(0.753, 0.749, 0.678, 1));
    skyLight->SetSpecularIntensity(0);
    skyLight->SetOccludee(false);
    skyLight->SetOccluder(false);
    skyLight->SetCastShadows(true);
    skyLight->SetShadowCascade(Urho3D::CascadeParameters(20, 50, 100, 500, 0.8f));
    skyLight->SetShadowFocus(Urho3D::FocusParameters(true, true, true, 1.0f, 5.0f));
    skyLight->SetShadowBias(Urho3D::BiasParameters(1e-005, 0.001));

    
  
    
    if (skyNode) {
        //ProcSky* procSky(skyNode->GetComponent<ProcSky>());
        if (procSky) {
            // Can set other parameters here; e.g., SetUpdateMode(), SetUpdateInterval(), SetRenderSize()
            procSky->Initialize();
            URHO3D_LOGINFO("ProcSky Initialized.");
        } else {
            URHO3D_LOGERROR("ProcSky node missing ProcSky component.");
        }
    } else {
        URHO3D_LOGERROR("ProcSky node not found in scene.");
    }
    
    
    
    
    // Create 1000 mushrooms in the terrain. Always face outward along the terrain normal
   /*
    const unsigned NUM_MUSHROOMS = 0;
    for (unsigned i = 0; i < NUM_MUSHROOMS; ++i)
    {
        Node* objectNode = scene_->CreateChild("SafetyCone");
        Vector3 position(Random(2000.0f) - 1000.0f, 0.0f, Random(2000.0f) - 1000.0f);
        position.y_ = terrain->GetHeight(position);
        objectNode->SetPosition(position);
        // Create a rotation quaternion from up vector to terrain normal
        objectNode->SetRotation(Quaternion(Vector3::UP, terrain->GetNormal(position)));
        objectNode->SetScale(3.0f);
        StaticModel* object = objectNode->CreateComponent<StaticModel>();
        object->SetModel(cache->GetResource<Model>("MyProjects/SafetyCone/SafetyCone.mdl"));
        object->SetMaterial(cache->GetResource<Material>("MyProjects/SafetyCone/ConeBase.xml"));
        object->SetMaterial(cache->GetResource<Material>("MyProjects/SafetyCone/SafetyCone.xml"));
        object->SetCastShadows(true);
        
        
        
        RigidBody* body = objectNode->CreateComponent<RigidBody>();
        //body->SetCollisionLayer(2);
        body->SetMass(2.0f);
        body->SetFriction(0.75f);
        CollisionShape* shape = objectNode->CreateComponent<CollisionShape>();
        //shape->SetTriangleMesh(object->GetModel(), 0);
        shape->SetConvexHull(object->GetModel(), 0);
    }
    */
}

void RacingGame::CreateVehicle()
{
    Node* vehicleNode = scene_->CreateChild("Vehicle");
    vehicleNode->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
    
    
    // Create the vehicle logic component
    vehicle_ = vehicleNode->CreateComponent<Vehicle>();
    
    // Create the rendering and physics components
    vehicle_->Init();
}

void RacingGame::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();
    
    // Construct new Text object, set string to display and font to use
    Text* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetText(
                             "Use WASD keys to drive, mouse/touch to rotate camera\n"
                             "F5 to save scene, F7 to load"
                             );
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    instructionText->SetTextAlignment(HA_CENTER);
    
    // Position the text relative to the screen center
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetVerticalAlignment(VA_CENTER);
    instructionText->SetPosition(0, ui->GetRoot()->GetHeight() / 4);
    
    
    
    // insert speed
    Text* speedText = ui->GetRoot()->CreateChild<Text>("UITextSpeed");
    speedText->SetText("x km/h");
    speedText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    speedText->SetTextAlignment(HA_CENTER);
    // Position the text relative to the screen center
    speedText->SetHorizontalAlignment(HA_CENTER);
    speedText->SetVerticalAlignment(VA_CENTER);
    speedText->SetPosition(ui->GetRoot()->GetWidth()/8, ui->GetRoot()->GetHeight() / 8);
    
    Text* gearText = ui->GetRoot()->CreateChild<Text>("UITextGear");
    gearText->SetText("x gear");
    gearText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    gearText->SetTextAlignment(HA_CENTER);
    // Position the text relative to the screen center
    gearText->SetHorizontalAlignment(HA_CENTER);
    gearText->SetVerticalAlignment(VA_CENTER);
    gearText->SetPosition(ui->GetRoot()->GetWidth()/8, ui->GetRoot()->GetHeight() / 6);
    
    Text* fpsText = ui->GetRoot()->CreateChild<Text>("UITextFPS");
    fpsText->SetText("x fps");
    fpsText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    fpsText->SetTextAlignment(HA_CENTER);
    // Position the text relative to the screen center
    fpsText->SetHorizontalAlignment(HA_CENTER);
    fpsText->SetVerticalAlignment(VA_CENTER);
    fpsText->SetPosition(-ui->GetRoot()->GetWidth()/2.7, -ui->GetRoot()->GetHeight()/2.1);
    
    
}

void RacingGame::SubscribeToEvents()
{
    // Subscribe to Update event for setting the vehicle controls before physics simulation
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(RacingGame, HandleUpdate));
    
    // Subscribe to PostUpdate event for updating the camera position after physics simulation
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(RacingGame, HandlePostUpdate));
    
    // Subscribe HandlePostRenderUpdate() function for processing the post-render update event, during which we request
    // debug geometry
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(RacingGame, HandlePostRenderUpdate));
    
    // Subscribe key down event
    SubscribeToEvent(E_KEYDOWN, URHO3D_HANDLER(RacingGame, HandleKeyDown));
    
    // Unsubscribe the SceneUpdate event from base class as the camera node is being controlled in HandlePostUpdate() in this sample
    // Subscribe scene update event
    SubscribeToEvent(E_SCENEUPDATE, URHO3D_HANDLER(RacingGame, HandleSceneUpdate));
}

void RacingGame::HandleKeyDown(StringHash eventType, VariantMap& eventData)
{
    using namespace KeyDown;
    
    int key = eventData[P_KEY].GetInt();
    
    // Close console (if open) or exit when ESC is pressed
    if (key == KEY_ESC)
    {
        engine_->Exit();
    }
    
    // toggle camera
    if (key == SDLK_F1)
    {
        switch (currrentViewport) {
            case 0:
                GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, cameraFreeNode_->GetComponent<Camera>()));
                URHO3D_LOGINFO("camera: Free");
                currrentViewport = 1;
                break;
            case 1:
                GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, cameraCarNode_->GetComponent<Camera>()));
                URHO3D_LOGINFO("camera: Car");
                currrentViewport = 0;
                break;
            default:
                break;
        }
        
    }
    
    
}

void RacingGame::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;
    
    Input* input = GetSubsystem<Input>();
    
    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;
    
    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);
    
    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraFreeNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
    
    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    // Use the Translate() function (default local space) to move relative to the node's orientation.
    if (input->GetKeyDown('U'))
        cameraFreeNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown('J'))
        cameraFreeNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown('H'))
        cameraFreeNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown('K'))
        cameraFreeNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);
}

void RacingGame::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;
    
    Input* input = GetSubsystem<Input>();
    
    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();
    
    if (vehicle_)
    {
        UI* ui = GetSubsystem<UI>();
        
        // Get movement controls and assign them to the vehicle component. If UI has a focused element, clear controls
        if (!ui->GetFocusElement())
        {
            
            vehicle_->controls_.Set(CTRL_FORWARD,   input->GetKeyDown('W'));
            vehicle_->controls_.Set(CTRL_BACK,      input->GetKeyDown('S'));
            vehicle_->controls_.Set(CTRL_LEFT,      input->GetKeyDown('A'));
            vehicle_->controls_.Set(CTRL_RIGHT,     input->GetKeyDown('D'));
            vehicle_->controls_.Set(CTRL_SPACE,     input->GetKeyDown(KEY_SPACE));
            
            
            


            
            vehicle_->controls_.yaw_ += (float)input->GetMouseMoveX() * YAW_SENSITIVITY;
            vehicle_->controls_.pitch_ += (float)input->GetMouseMoveY() * YAW_SENSITIVITY;
            
            // Limit pitch
            vehicle_->controls_.pitch_ = Clamp(vehicle_->controls_.pitch_, 0.0f, 80.0f);
            
            
        /*
            // Check for loading / saving the scene
            if (input->GetKeyPress(KEY_F5))
            {
                File saveFile(context_, GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Scenes/RacingGame.xml",
                              FILE_WRITE);
                scene_->SaveXML(saveFile);
            }
       
            if (input->GetKeyPress(KEY_F7))
            {
                File loadFile(context_, GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Scenes/RacingGame.xml", FILE_READ);
                scene_->LoadXML(loadFile);
                // After loading we have to reacquire the weak pointer to the Vehicle component, as it has been recreated
                // Simply find the vehicle's scene node by name as there's only one of them
                Node* vehicleNode = scene_->GetChild("Vehicle", true);
                if (vehicleNode)
                    vehicle_ = vehicleNode->GetComponent<Vehicle>();
            }
            */
        }
        else
            vehicle_->controls_.Set(CTRL_FORWARD | CTRL_BACK | CTRL_LEFT | CTRL_RIGHT | CTRL_SPACE, false);
    }
    

    // set vehicle speed to UI
    Text* speedText = (Text*)GetSubsystem<UI>()->GetRoot()->GetChild("UITextSpeed",true);
    speedText->SetText(String((int)vehicle_->getSpeed()) + " km/h");
    
    // set vehicle gear to UI
    Text* gearText = (Text*)GetSubsystem<UI>()->GetRoot()->GetChild("UITextGear",true);
    gearText->SetText(String((int)vehicle_->getGear()) + " gear");
    
    
    // set vehicle gear to UI
    Text* fpsText = (Text*)GetSubsystem<UI>()->GetRoot()->GetChild("UITextFPS",true);
    FrameInfo frameInfo = GetSubsystem<Renderer>()->GetFrameInfo();
    fpsText->SetText("FPS: " + String(1.0 / frameInfo.timeStep_));

    
    // Move the camera, scale movement with time step
    MoveCamera(timeStep);
    
    
    
}

void RacingGame::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
    if (!vehicle_)
        return;
    
    Node* vehicleNode = vehicle_->GetNode();
    
    // Physics update has completed. Position camera behind vehicle
    Quaternion dir(vehicleNode->GetRotation().YawAngle(), Vector3::UP);
    dir = dir * Quaternion(vehicle_->controls_.yaw_, Vector3::UP);
    dir = dir * Quaternion(vehicle_->controls_.pitch_, Vector3::RIGHT);
    
    Vector3 cameraTargetPos = vehicleNode->GetPosition() - dir * Vector3(0.0f, 0.0f, CAMERA_DISTANCE);
    Vector3 cameraStartPos = vehicleNode->GetPosition();
    
    
    // <<<< works not perfect (camera moves under the vehicle)
    // Raycast camera against static objects (physics collision mask 2)
    // and move it closer to the vehicle if something in between
    /*
    Ray cameraRay(cameraStartPos, cameraTargetPos - cameraStartPos);
    float cameraRayLength = (cameraTargetPos - cameraStartPos).Length();
    PhysicsRaycastResult result;
    scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, cameraRay, cameraRayLength, 2);
    if (result.body_)
        cameraTargetPos = cameraStartPos + cameraRay.direction_ * (result.distance_ - 0.5f);
    */
    cameraCarNode_->SetPosition(cameraTargetPos);
    cameraCarNode_->SetRotation(dir);
    
    

    
    
    
    //URHO3D_LOGINFOF( "dirPitch: %s ", result.distance_.ToString().CString());
    //URHO3D_LOGINFOF( "direction_: %s ", cameraRay.direction_.ToString().CString());
    
    
    
    // update sun
    Node* skyNode = scene_->GetChild("ProcSkyNode");
    Node* skyLightNode = skyNode->GetChild("ProcSkyLight");
    Light* skyLight = skyLightNode->GetComponent<Light>();
    //URHO3D_LOGINFOF( "Sun Pos: %f ", (skyLightNode->GetRotation()).PitchAngle());
    //URHO3D_LOGINFOF( "Sun TEstPos: %f ", ((skyLightNode->GetRotation()).PitchAngle())/10);
    //URHO3D_LOGINFOF( "Sun Brightness: %f", skyLight->GetBrightness());
    if( (skyLightNode->GetRotation()).PitchAngle() < 10 ){
        skyLight->SetBrightness( (skyLightNode->GetRotation()).PitchAngle()/10.0);
    } else {
        skyLight->SetBrightness(1);
    }
        
    
}


void RacingGame::HandleSceneUpdate(StringHash eventType, VariantMap& eventData)
{
    // Move the camera by touch, if the camera node is initialized by descendant sample class
    /*
    if (cameraCarNode_)
    {
        Input* input = GetSubsystem<Input>();
        for (unsigned i = 0; i < input->GetNumTouches(); ++i)
        {
            TouchState* state = input->GetTouch(i);
            if (!state->touchedElement_)    // Touch on empty space
            {
                if (state->delta_.x_ ||state->delta_.y_)
                {
                    Camera* camera = cameraCarNode_->GetComponent<Camera>();
                    if (!camera)
                        return;
                    
                    Graphics* graphics = GetSubsystem<Graphics>();
                   // yaw_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.x_;
                   // pitch_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.y_;
                    
                    // Construct new orientation for the camera scene node from yaw and pitch; roll is fixed to zero
                   // cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
                }
                else
                {
                    // Move the cursor to the touch position
                    Cursor* cursor = GetSubsystem<UI>()->GetCursor();
                    if (cursor && cursor->IsVisible())
                        cursor->SetPosition(state->position_);
                }
            }
        }
    }
     */
}

void RacingGame::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    // If draw debug mode is enabled, draw navigation mesh debug geometry
    if (drawDebug_){
        scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);
    }

    

    
  /*
    if (currentPath_.Size())
    {
        // Visualize the current calculated path
        DebugRenderer* debug = scene_->GetComponent<DebugRenderer>();
        debug->AddBoundingBox(BoundingBox(endPos_ - Vector3(0.1f, 0.1f, 0.1f), endPos_ + Vector3(0.1f, 0.1f, 0.1f)),
                              Color(1.0f, 1.0f, 1.0f));
        
        // Draw the path with a small upward bias so that it does not clip into the surfaces
        Vector3 bias(0.0f, 0.05f, 0.0f);
        debug->AddLine(jackNode_->GetPosition() + bias, currentPath_[0] + bias, Color(1.0f, 1.0f, 1.0f));
        
        if (currentPath_.Size() > 1)
        {
            for (unsigned i = 0; i < currentPath_.Size() - 1; ++i)
                debug->AddLine(currentPath_[i] + bias, currentPath_[i + 1] + bias, Color(1.0f, 1.0f, 1.0f));
        }
    }
   */
}
