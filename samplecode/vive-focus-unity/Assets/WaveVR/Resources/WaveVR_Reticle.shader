Shader "WaveVR/Reticle" {
  Properties {
    _InnerDiameter ("InnerDiameter", Range(0, 10.0)) = 2.0
    _OuterDiameter ("OuterDiameter", Range(0.01, 10.0)) = 3.0
    _DistanceInMeters ("DistanceInMeters", Range(0.0, 100.0)) = 3.0
    _Color ("Color", Range(0.0, 1.0)) = 1.0
    _TriggerProgress ("ProgressSwitch", Range(0.0 , 1.0)) = 0.0
  }

  SubShader {
    Tags { "Queue"="Overlay" "IgnoreProjector"="True" "RenderType"="Transparent" }
    Pass {
      Blend SrcAlpha OneMinusSrcAlpha
      AlphaTest Off
      Cull Off
      ZWrite Off
      ZTest Always

      CGPROGRAM

      #pragma vertex vert
      #pragma fragment frag

      #include "UnityCG.cginc"

      uniform float _InnerDiameter;
      uniform float _OuterDiameter;
      uniform float _DistanceInMeters;
      uniform float _TriggerProgress;
      uniform float colorRotFactor[4];
      uniform fixed4 _Color;

      struct vertIn {
        float4 vertex : POSITION;
      };

      struct fragIn {
          fixed4 color : COLOR;
          float4 position : SV_POSITION;
      };

      fragIn vert(vertIn vi) {
        float scale = lerp(_OuterDiameter, _InnerDiameter, vi.vertex.z);

        float4 vert_out = float4(vi.vertex.x * scale, vi.vertex.y * scale, _DistanceInMeters, 1.0);

        fragIn o;
        o.position = UnityObjectToClipPos (vert_out);
        o.color.rgb = _Color.rgb;
        o.color.a = _TriggerProgress == 1.0 ? _Color.a * ((vi.vertex.x * (colorRotFactor[0] - colorRotFactor[2])) + (vi.vertex.y * (colorRotFactor[3] - colorRotFactor[1]))) : _Color.a;
        return o;
      }

      fixed4 frag(fragIn fi) : SV_Target {
        return fi.color;
      }

      ENDCG
    }
  }
}
