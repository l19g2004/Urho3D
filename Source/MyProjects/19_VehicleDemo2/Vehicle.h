    #pragma once

    #include <Urho3D/Input/Controls.h>

    #include <Urho3D/Scene/LogicComponent.h>
    #include <Bullet/BulletDynamics/Vehicle/btRaycastVehicle.h>

    namespace Urho3D
    {
    class Constraint;
    class Node;
    class RigidBody;
    }

    using namespace Urho3D;


    //=============================================================================
    //=============================================================================
    const int CTRL_FORWARD = 1;
    const int CTRL_BACK = 2;
    const int CTRL_LEFT = 4;
    const int CTRL_RIGHT = 8;
    const int CTRL_SPACE = 10;

    const float YAW_SENSITIVITY = 0.1f;
    const float ENGINE_POWER = 10.0f;
    const float DOWN_FORCE = 10.0f;
    const float MAX_WHEEL_ANGLE = 22.5f;

    //=============================================================================
    // Vehicle component, responsible for physical movement according to controls.
    //=============================================================================
    class Vehicle : public LogicComponent
    {
        URHO3D_OBJECT(Vehicle, LogicComponent)


    public:
        /// Construct.
        Vehicle(Context* context);
        ~Vehicle();
       
        /// Register object factory and attributes.
        static void RegisterObject(Context* context);
       
        /// Perform post-load after deserialization. Acquire the components from the scene nodes.
        virtual void ApplyAttributes();

        /// Initialize the vehicle. Create rendering and physics components. Called by the application.
        void Init();

        /// Handle physics world update. Called by LogicComponent base class.
        virtual void FixedUpdate(float timeStep);
        virtual void PostUpdate(float timeStep);
       
        /// Movement controls.
        Controls controls_;
       
    private:
        // Hull RigidBody
        WeakPtr<RigidBody> hullBody_;
       
        /// Current left/right steering amount (-1 to 1.)
        float steering_;

        // raycast vehicle
        btRaycastVehicle::btVehicleTuning   m_tuning;
        btVehicleRaycaster                  *m_vehicleRayCaster;
        btRaycastVehicle                    *m_vehicle;

        // IDs of the wheel scene nodes for serialization.
        Vector<Node*>           m_vpNodeWheel;

        float   gEngineForce;
        float   gBreakingForce;

        float   maxEngineForce;
        float   maxBreakingForce;

        float   gVehicleSteering;
        float   steeringIncrement;
        float   steeringClamp;
        float   wheelRadius;
        float   wheelWidth;
        float   wheelFriction;
        float   suspensionStiffness;
        float   suspensionDamping;
        float   suspensionCompression;
        float   rollInfluence;
        float   suspensionRestLength;

    };


