#include <Urho3D/Urho3D.h>

#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/DebugRenderer.h>

#include "Vehicle.h"

#include <SDL/SDL_log.h>
#include <Bullet/BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <Bullet/BulletDynamics/Dynamics/btDynamicsWorld.h>

//=============================================================================
//=============================================================================
#define CUBE_HALF_EXTENTS   1
#define DELETE_NULL(x)      { if (x) delete x; x = NULL; }

//=============================================================================
//=============================================================================
Vehicle::Vehicle(Context* context)
: LogicComponent( context )
, steering_( 0.0f )
{
    // fixed update() for inputs and post update() to sync wheels for rendering
    SetUpdateEventMask( USE_FIXEDUPDATE | USE_POSTUPDATE );
    
    gEngineForce = 0.0f;          //applied force (real time)
    gBreakingForce = 0.0f;        //applied force (real time)
    maxEngineForce = 5000.f;    //this should be engine/velocity dependent
    maxBreakingForce = 500.f;
    
    //gVehicleSteering = 0.0f;
    steeringIncrement = 0.05f;    //amount of applied angular force key:A/D
    steeringClamp = 0.2f;   	 //Don't increase the angle
    wheelRadius = 0.25f;
    wheelWidth = 0.2f;
    wheelFriction = 100;//BT_LARGE_FLOAT;
    suspensionStiffness = 40.0f;//20.f;
    suspensionDamping = 2.3f;//2.3f;
    suspensionCompression = 4.4f;//4.4f;
    rollInfluence = 0.01f;//1.0f;
    suspensionRestLength = 0.6f;//0.6
    
    m_vehicleRayCaster = NULL;
    m_vehicle = NULL;
    
    m_vpNodeWheel.Clear();
}

//=============================================================================
//=============================================================================
Vehicle::~Vehicle()
{
    DELETE_NULL( m_vehicleRayCaster );
    DELETE_NULL( m_vehicle );
    
    m_vpNodeWheel.Clear();
}

//=============================================================================
//=============================================================================
void Vehicle::RegisterObject(Context* context)
{
    context->RegisterFactory<Vehicle>();
    
    
    URHO3D_ATTRIBUTE("Controls Yaw", float, controls_.yaw_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Controls Pitch", float, controls_.pitch_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Steering", float, steering_, 0.0f, AM_DEFAULT);
    
}

//=============================================================================
//=============================================================================
void Vehicle::ApplyAttributes()
{
}

//=============================================================================
//=============================================================================
void Vehicle::Init()
{
    // This function is called only from the main program when initially creating the vehicle, not on scene load
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    
    StaticModel* hullObject = node_->CreateComponent<StaticModel>();
    hullBody_ = node_->CreateComponent<RigidBody>();
    CollisionShape* hullColShape = node_->CreateComponent<CollisionShape>();
    
    hullBody_->SetMass(800.0f);
    hullBody_->SetLinearDamping(0.2f); // Some air resistance
    hullBody_->SetAngularDamping(0.5f);
    hullBody_->SetCollisionLayer(1);
    
    int rightIndex = 0;
    int upIndex = 1;
    int forwardIndex = 2;
    Scene* scene = GetScene();
    PhysicsWorld *pPhysWorld = scene->GetComponent<PhysicsWorld>();
    btDynamicsWorld *pbtDynWorld = (btDynamicsWorld*)pPhysWorld->GetWorld();
    
    m_vehicleRayCaster = new btDefaultVehicleRaycaster( pbtDynWorld );
    m_vehicle = new btRaycastVehicle( m_tuning, hullBody_->GetBody(), m_vehicleRayCaster );
    pbtDynWorld->addVehicle( m_vehicle );
    
    m_vehicle->setCoordinateSystem( rightIndex, upIndex, forwardIndex );
    
    node_->SetScale( Vector3(1.0f, 1.0f, 1.0f) );
    //Vector3 v3BoxExtents = Vector3::ONE;//Vector3(1.5f, 1.0f, 3.0f);
    hullObject->SetModel(cache->GetResource<Model>("MyProjects/MiniCooper/test/Chassis_001.mdl"));
    hullColShape->SetBox((hullObject->GetBoundingBox()).Size());

    
    //hullObject->SetMaterial(cache->GetResource<Material>("Materials/Stone.xml"));
    hullObject->SetCastShadows(true);




    
  
    


    //float connectionHeight = -0.4f;//1.2f;
    float connectionHeight = 0.0f;
    bool isFrontWheel=true;
    btVector3 wheelDirectionCS0(0,-1,0);
    btVector3 wheelAxleCS(-1,0,0);
    
    
    // front right
    //////////////
    Node* objectNode0 = GetScene()->CreateChild("wheel_001");
    StaticModel* object0 = objectNode0->CreateComponent<StaticModel>();
    object0->SetModel(cache->GetResource<Model>("MyProjects/MiniCooper/test/wheel_000.mdl"));
    object0->SetCastShadows(true);
    
    btVector3 connectionPointCS0(((object0->GetBoundingBox()).Center()).x_, ((object0->GetBoundingBox()).Center()).y_, ((object0->GetBoundingBox()).Center()).z_);
    wheelRadius = object0->GetBoundingBox().HalfSize().y_;
    m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
    
    
    
    SDL_Log( "objectNode0: %f, %f, %f \n", ((object0->GetBoundingBox()).Center()).x_, ((object0->GetBoundingBox()).Center()).y_, ((object0->GetBoundingBox()).Center()).z_);
    
    //SDL_Log( "objectNode0.HalfSize: %s \n", object0->GetBoundingBox().HalfSize().ToString().CString() );

    

    // front left
    /////////////
    Node* objectNode1 = GetScene()->CreateChild("wheel_002");
    StaticModel* object1 = objectNode1->CreateComponent<StaticModel>();
    object1->SetModel(cache->GetResource<Model>("MyProjects/MiniCooper/test/wheel_001.mdl"));
    object1->SetCastShadows(true);
    
    connectionPointCS0 = btVector3(((object1->GetBoundingBox()).Center()).x_, ((object1->GetBoundingBox()).Center()).y_+0.3, ((object1->GetBoundingBox()).Center()).z_);
    wheelRadius = object1->GetBoundingBox().HalfSize().y_;
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning, isFrontWheel);
    
    
    
    
    isFrontWheel = false;
    
    
    // back right
    /////////////
    Node* objectNode2 = GetScene()->CreateChild("wheel_003");
    StaticModel* object2 = objectNode2->CreateComponent<StaticModel>();
    object2->SetModel(cache->GetResource<Model>("MyProjects/MiniCooper/test/wheel_002.mdl"));
    object2->SetCastShadows(true);
    connectionPointCS0 = btVector3(((object2->GetBoundingBox()).Center()).x_, ((object2->GetBoundingBox()).Center()).y_, ((object2->GetBoundingBox()).Center()).z_);
    wheelRadius = object2->GetBoundingBox().HalfSize().y_;
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
    
    
    
    // back left
    /////////////
    Node* objectNode3 = GetScene()->CreateChild("wheel_004");
    StaticModel* object3 = objectNode3->CreateComponent<StaticModel>();
    object3->SetModel(cache->GetResource<Model>("MyProjects/MiniCooper/test/wheel_003.mdl"));
    object3->SetCastShadows(true);
    
    connectionPointCS0 = btVector3(((object3->GetBoundingBox()).Center()).x_, ((object3->GetBoundingBox()).Center()).y_, ((object3->GetBoundingBox()).Center()).z_);
    wheelRadius = object3->GetBoundingBox().HalfSize().y_;
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
    
    
    
    
    
    //connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3f*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
    //m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
    
    
    
    
    
    for ( int i = 0; i < m_vehicle->getNumWheels(); i++ )
    {
        btWheelInfo& wheel = m_vehicle->getWheelInfo( i );
        wheel.m_suspensionStiffness = suspensionStiffness;
        wheel.m_wheelsDampingRelaxation = suspensionDamping;
        wheel.m_wheelsDampingCompression = suspensionCompression;
        wheel.m_frictionSlip = wheelFriction;
        wheel.m_rollInfluence = rollInfluence;
    }
    
    if ( m_vehicle )
    {
        m_vehicle->resetSuspension();
        
        for ( int i = 0; i < m_vehicle->getNumWheels(); i++ )
        {
            //synchronize the wheels with the (interpolated) chassis worldtransform
            m_vehicle->updateWheelTransform(i,true);
            
            btTransform transform = m_vehicle->getWheelTransformWS( i );
            
            
            
            //Vector3 v3Origin = ToVector3( transform.getOrigin() );
            //Quaternion qRot = ToQuaternion( transform.getRotation() );
            
            // create wheel node
            Node *wheelNode = GetScene()->CreateChild();
     
            
            //wheelNode->SetPosition( v3Origin );

            //btWheelInfo whInfo = m_vehicle->getWheelInfo( i );
            //Vector3 v3PosLS = ToVector3( whInfo.m_chassisConnectionPointCS );
            

            
            //wheelNode->SetRotation( v3PosLS.x_ >= 0.0 ? Quaternion(0.0f, 0.0f, -90.0f) : Quaternion(0.0f, 0.0f, 90.0f) );
            //wheelNode->SetScale(Vector3(1.0f, 0.65f, 1.0f));
            
            StaticModel *pWheel = wheelNode->CreateComponent<StaticModel>();
            if(i == 0)
                pWheel->SetModel(cache->GetResource<Model>("MyProjects/SimpeCar/wheel.mdl"));
            else
                pWheel->SetModel(cache->GetResource<Model>("MyProjects/SimpeCar/wheel.mdl"));
            
            if(i == 0) {
                //wheelNode->SetPosition( Vector3(50,50,50) );
                //wheelNode->SetRotation(Quaternion(0.0f, 0.0f, 0.0f));
            }else {
                //wheelNode->SetPosition( v3Origin );
                //wheelNode->SetRotation(Quaternion(0.0f, 0.0f, 0.0f));
            }
            
            m_vpNodeWheel.Push( wheelNode );
            
            //pWheel->SetModel(cache->GetResource<Model>("MyProjects/MiniCooper/test/wheel_004.mdl"));
            pWheel->SetMaterial(cache->GetResource<Material>("MyProjects/SimpeCar/TireTextureMaterial.xml"));
            pWheel->SetCastShadows(true);
        }
    }
}

//=============================================================================
//=============================================================================
void Vehicle::FixedUpdate(float timeStep)
{
    
    //SDL_Log( "gVehicleSteering: %f\n", gVehicleSteering);
    //SDL_Log( "controls_.buttons_: %d\n", controls_.buttons_);
    
    
    // Read controls
    if(controls_.buttons_ & CTRL_LEFT){
        gVehicleSteering -= steeringIncrement;
        if(gVehicleSteering < -steeringClamp){gVehicleSteering = -steeringClamp;}
    } else if(controls_.buttons_ & CTRL_RIGHT){
        gVehicleSteering += steeringIncrement;
        if( gVehicleSteering > +steeringClamp){gVehicleSteering = +steeringClamp;}
    }else{
        gVehicleSteering = 0;
    }
    
    if( (controls_.buttons_ & CTRL_FORWARD) || (controls_.buttons_ & CTRL_BACK) ) {
        if(controls_.buttons_ & CTRL_FORWARD){
            gEngineForce = maxEngineForce;
        }
        if(controls_.buttons_ & CTRL_BACK){
            gEngineForce = -maxEngineForce;
        }
        gBreakingForce = 0.0f;
    } else {
        gEngineForce = 0.0f;
    }
    
    if(controls_.buttons_ & CTRL_SPACE){
        gBreakingForce = maxBreakingForce;
    } else {
        gBreakingForce = 0.0f;
    }
    
    /*
     // When steering, wake up the wheel rigidbodies so that their orientation is updated
     if ( newSteering != 0.0f )
     {
     steering_ = steering_ * 0.95f + newSteering * 0.05f;
     }
     else
     {
     steering_ = steering_ * 0.95f + newSteering * 0.05f;
     }
     */
    
    int wheelIndex = 2;
    m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
    m_vehicle->setBrake(gBreakingForce,wheelIndex);
    wheelIndex = 3;
    m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
    m_vehicle->setBrake(gBreakingForce,wheelIndex);
    
    wheelIndex = 0; // rechts
    m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
    wheelIndex = 1; // links
    m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
    
}

//=============================================================================
// sync wheels for rendering
//=============================================================================
void Vehicle::PostUpdate(float )
{
    for ( int i = 0; i < m_vehicle->getNumWheels(); i++ )
    {
        m_vehicle->updateWheelTransform( i, true );
        
        btTransform transform = m_vehicle->getWheelTransformWS( i );

        Vector3 v3Origin = ToVector3( transform.getOrigin() );
        Quaternion qRot = ToQuaternion( transform.getRotation() );
        
        
/*
        ResourceCache* cache = GetSubsystem<ResourceCache>();
        Node* axesNode = GetScene()->CreateChild("Axes");
        StaticModel* axesModel = axesNode->CreateComponent<StaticModel>();
        axesModel->SetModel(cache->GetResource<Model>("MyProjects/Helpers/Axes.mdl"));
        axesModel->SetMaterial(cache->GetResource<Material>("MyProjects/Helpers/blue.xml"));
        axesNode->SetPosition(v3Origin);
        axesNode->SetScale(3.0f);
*/
       

        
        Node *pWheel = m_vpNodeWheel[ i ];

        
        btWheelInfo whInfo = m_vehicle->getWheelInfo( i );
        Vector3 v3PosLS = ToVector3( whInfo.m_chassisConnectionPointCS );
 
        if(i == 0) {
            pWheel->SetPosition(Vector3(-0.704,-0.310,-1.189));
            

            SDL_Log("##################### \n");
            
            //SDL_Log("transform: %f, %f, %f \n", v3Origin.x_, v3Origin.y_, v3Origin.z_);
            
            //v3Origin = Vector3(-0.704,-0.310,-1.189);
            
            //SDL_Log("transform: %f, %f, %f \n", v3Origin.x_, v3Origin.y_, v3Origin.z_);
            
            
            
            SDL_Log("qRot: %f, %f, %f \n",  qRot.PitchAngle(), qRot.YawAngle(), qRot.RollAngle());
            
            qRot.FromEulerAngles(10.0, 15.0, 20.0);
            
            SDL_Log("qRot: %f, %f, %f \n",  qRot.PitchAngle(), qRot.YawAngle(), qRot.RollAngle());
            
            
            
            

            pWheel->SetRotation(qRot);
            //pWheel->SetRotation(Quaternion(0.0f, 90.0f, 0.0f)*qRot);
            //SDL_Log( "v3Origin: \"%s\" \n", (v3Origin.ToString()).CString());
            //SDL_Log( "qRot: \"%s\" \n", pWheel->GetRotation().ToString().CString());
        } else{
            pWheel->SetPosition( v3Origin );
            Quaternion qRotator = ( v3PosLS.x_ >= 0.0 ? Quaternion(0.0f, 0.0f, -90.0f) : Quaternion(0.0f, 0.0f, 90.0f) );
            pWheel->SetRotation( qRot * qRotator );
        
          
        }
        
    }
    
}

