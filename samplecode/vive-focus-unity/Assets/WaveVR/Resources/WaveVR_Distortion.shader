Shader "WaveVR/Distortion"
{
	Properties
	{
		_MainTex ("Texture", 2D) = "white" {}
	}
	SubShader
	{
		Tags { "RenderType"="Opaque" }
		LOD 100

		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			
			#include "UnityCG.cginc"

			struct appdata
			{
				float4 vertex : POSITION;
				float2 uvr : TEXCOORD0;
				float2 uvg : TEXCOORD1;
				float2 uvb : TEXCOORD2;
			};

			struct v2f
			{
				float2 uvr : TEXCOORD0;
				float2 uvg : TEXCOORD1;
				float2 uvb : TEXCOORD2;
				float4 vertex : SV_POSITION;
			};

			sampler2D _MainTex;
			float4 _MainTex_ST;
			
			v2f vert (appdata v)
			{
				v2f o;
				o.vertex = v.vertex;
				o.uvr = v.uvr;
				o.uvg = v.uvg;
				o.uvb = v.uvb;
				return o;
			}
			
			fixed4 frag (v2f i) : SV_Target
			{
				if (i.uvg.x < 0 || i.uvg.y < 0 || i.uvg.x > 1 || i.uvg.y > 1)
					return float4(0, 0, 0, 0);
				// sample the texture
				float r = tex2D(_MainTex, i.uvr).x;
				float g = tex2D(_MainTex, i.uvg).y;
				float b = tex2D(_MainTex, i.uvb).z;
				fixed4 col = float4(r, g, b, 0);
				return col;
			}
			ENDCG
		}
	}
}
