using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

public struct GameTime
{
    /// <summary>
    /// 帧间隔时间
    /// </summary>
    public float tickInterval { get; private set; }
}

public struct GlobalGameTime : IComponentData
{
    public GameTime gameTime;//当前游戏时间
    public float frameDuration;//当前帧的长度
}
