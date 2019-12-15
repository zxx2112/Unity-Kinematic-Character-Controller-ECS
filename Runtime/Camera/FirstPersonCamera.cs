using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FirstPersonCamera : MonoBehaviour
{
    [SerializeField] private Transform player;
    [SerializeField] private bool syncPosition;
    [SerializeField] private float yaw;
    [SerializeField] private float pitch;
    [SerializeField] private float pitchSpeed = 1;
    [SerializeField] private float interpolateSpeed = 1;

    void Start()
    {
        
    }

    void Update()
    {
        if(syncPosition) {
            var playerPosition = GetPlayerPosition();
            if (playerPosition.HasValue)
                transform.position = playerPosition.Value;
        }

        transform.rotation = Quaternion.Euler(pitch, yaw, 0);
    }

    private void LateUpdate() {
        //刷新Pitch
        pitch = GetPitch();
        //刷新Yaw
        yaw = GetYaw();
    }
    
    //获取Y轴旋转
    private float GetYaw() {
        if (player == null) return 0;

        var ret = player.eulerAngles.y;
        ret = Mathf.LerpAngle(yaw, ret,Mathf.Clamp01(Time.deltaTime * interpolateSpeed));

        return ret;
    }

    private Vector3? GetPlayerPosition() {
        if (player == null) return null;

        return player.position;
    }

    private float GetPitch() {
        var ret = pitch - Input.GetAxisRaw("Mouse Y") * pitchSpeed;

        return ret;
    }
}
