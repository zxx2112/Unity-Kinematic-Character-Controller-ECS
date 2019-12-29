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
    public int tick;
    public float tickDuration;

    public GameTime(int tickRate) {
        this.m_tickRate = tickRate;
        this.tickInterval = 1.0f / m_tickRate;
        this.tick = 1;
        this.tickDuration = 0;
    }

    private int m_tickRate;
}

public struct GlobalGameTime : IComponentData
{
    public GameTime gameTime;//当前游戏时间
    public float frameDuration;//当前帧的长度
}
