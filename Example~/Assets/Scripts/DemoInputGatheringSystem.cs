using Unity.Entities;
using UnityEngine;
using UnityEngine.InputSystem;

[AlwaysUpdateSystem]
[UpdateBefore(typeof(CharacterControllerOneToManyInputSystem))]
class DemoInputGatheringSystem : ComponentSystem,
    InputActions.ICharacterControllerActions
{
    InputActions m_InputActions;

    EntityQuery m_CharacterControllerInputQuery;
    EntityQuery m_CharacterGunInputQuery;

    Vector2 m_CharacterMovement;
    Vector2 m_CharacterLooking;
    float m_CharacterFiring;
    bool m_CharacterJumped;

    EntityQuery m_VehicleInputQuery;

    Vector2 m_VehicleLooking;
    Vector2 m_VehicleSteering;
    float m_VehicleThrottle;
    int m_VehicleChanged;

    protected override void OnCreate()
    {
        m_InputActions = new InputActions();
        m_InputActions.CharacterController.SetCallbacks(this);

        m_CharacterControllerInputQuery = GetEntityQuery(typeof(CharacterControllerInput));
    }

    protected override void OnStartRunning() => m_InputActions.Enable();

    protected override void OnStopRunning() => m_InputActions.Disable();

    protected override void OnUpdate()
    {
        // character controller
        if (m_CharacterControllerInputQuery.CalculateEntityCount() == 0)
            EntityManager.CreateEntity(typeof(CharacterControllerInput));

        m_CharacterControllerInputQuery.SetSingleton(new CharacterControllerInput
        {
            //Looking = m_CharacterLooking,
            Movement = m_CharacterMovement,
            Jumped = m_CharacterJumped ? 1 : 0
        });

        m_CharacterJumped = false;

        m_VehicleChanged = 0;
    }

    void InputActions.ICharacterControllerActions.OnMove(InputAction.CallbackContext context) => m_CharacterMovement = context.ReadValue<Vector2>();
    void InputActions.ICharacterControllerActions.OnLook(InputAction.CallbackContext context) => m_CharacterLooking = context.ReadValue<Vector2>();
    void InputActions.ICharacterControllerActions.OnFire(InputAction.CallbackContext context) => m_CharacterFiring = context.ReadValue<float>();
    void InputActions.ICharacterControllerActions.OnJump(InputAction.CallbackContext context) { if (context.started) m_CharacterJumped = true; }
}
