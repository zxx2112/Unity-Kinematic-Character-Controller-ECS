using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class CharacterControllerUtilities
{
    public enum CharacterSupportState : byte
    {
        Unsupported = 0,
        Sliding,
        Supported
    }
}
