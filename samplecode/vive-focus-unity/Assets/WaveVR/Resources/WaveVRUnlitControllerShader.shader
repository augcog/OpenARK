// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."


// Color fade out if the vertex is close to eye.
Shader "WaveVR/UnlitControllerShader"
{
	Properties
	{
		_MainTex ("Texture", 2D) = "white" {}
		_FadeAlpha ("Fading alpha value", Range(0.0, 1.0)) = 1.0
		_BaseAlpha ("Minimum alpha value.", Range(0.0, 1.0)) = 0.1
		_OpaqueDistance ("Keep opaque from this distance", Range(0.001, 1.0)) = 0.15
		_MinDistance ("Keep BaseAlpha from this distance", Range(0.001, 1.0)) = 0.1
	}

	SubShader
	{
		Tags { "RenderType" = "Transparent" "Queue" = "Transparent+100" }
		LOD 100

		Pass
		{
			Blend SrcAlpha OneMinusSrcAlpha
			Lighting Off

			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			
			#include "UnityCG.cginc"

			struct appdata
			{
				float4 vertex : POSITION;
				float2 uv : TEXCOORD0;
			};

			struct v2f
			{
				float2 uv : TEXCOORD0;
				float4 vertex : SV_POSITION;
				float4 col : COLOR0;
			};

			sampler2D _MainTex;
			float4 _MainTex_ST;
			float _FadeAlpha;
			float _BaseAlpha;
			float _OpaqueDistance;
			float _MinDistance;
			
			v2f vert (appdata v)
			{
				v2f o;
				o.vertex = UnityObjectToClipPos(v.vertex);
				o.uv = TRANSFORM_TEX(v.uv, _MainTex);

				// The distance to eye.  The more, the opaque.
				float distance = abs(length(UnityObjectToViewPos(v.vertex)));
				float alpha;
				if (distance < _MinDistance)
					alpha = _BaseAlpha;
				else
					alpha = (1 - clamp((_OpaqueDistance - distance + _MinDistance) / (_OpaqueDistance - _MinDistance), 0.0, 1.0));
				float base = _BaseAlpha + alpha * (1 - _BaseAlpha);
				o.col = float4(0, 0, 0, min(_FadeAlpha, base));
				return o;
			}
			
			fixed4 frag (v2f i) : SV_Target
			{
				// sample the texture
				fixed4 col = tex2D(_MainTex, i.uv);
				col.a = col.a * i.col.a;

				return col;
			}
			ENDCG
		}
	}
}
