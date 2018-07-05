Shader "WaveVR/TextShader"
{
	Properties{
		_MainTex("Font Texture", 2D) = "white" {}
		_Color("Text Color", Color) = (1,1,1,1)
		_BColor("Back Color", Color) = (0.5,0.5,0.5,0.2)
	}

	SubShader{
		Tags{ "Queue" = "Transparent" "IgnoreProjector" = "True" "RenderType" = "Transparent" }
		Lighting Off Cull Off ZWrite On Fog{ Mode Off }
		Blend SrcAlpha OneMinusSrcAlpha
		Pass{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#include "UnityCG.cginc"
			struct appdata
			{
				float4 vertex : POSITION; // vertex position
				float2 uv : TEXCOORD0; // texture coordinate
			};
			struct v2f
			{
				float2 uv : TEXCOORD0;
				float4 vertex : SV_POSITION;
			};

			float4 _MainTex_ST;

			v2f vert(appdata v)
			{
				v2f o;
				// transform position to clip space
				// (multiply with model*view*projection matrix)
				o.vertex = UnityObjectToClipPos(v.vertex);
				// just pass the texture coordinate
				o.uv = v.uv;
				return o;
			}

			sampler2D _MainTex;
			fixed4 _Color;
			fixed4 _BColor;

			fixed4 frag(v2f i) : SV_Target
			{
				// TODO: outline the text
				fixed4 col = tex2D(_MainTex, i.uv);
				if (col.a > 0)
					col.rgb = _Color.rgb;
				else
					clip(-1);
				return col;
			}
			ENDCG
		}
	}
}
