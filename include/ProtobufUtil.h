#pragma once
#include "Version.h"
#include "Hand.h"
#include "ProtobufHand.pb.h"

using namespace ark;

namespace protob {

    namespace util {
			/**
			* serialize hand
			* @param hands
			* @return a string of serialized hands
			*/
			std::string SerializeHand(std::vector<ark::Hand::Ptr> hands);

			/**
			* convert Vec3f to pointXYZ
			* @param Vec3f vec3fXYZ
			* @return PointXYZ
			*/
			protob::PointXYZ ConvertVec3fToPointXYZ(Vec3f vec3fXYZ);

			/**
			* convert point2i to pointIJ
			* @param point2i
			* @return PointIJ
			*/
			protob::PointIJ ConvertPoint2iToPointIJ(Point2i point2i);

			/**
			* serialize fingers
			* @param fingersXYZ to serialize
			* @param fingersIJ to serialize
			* @param protobOneHand to receive 
			*/
			void SerializeHandFingers(std::vector<Vec3f> fingersXYZ,
				std::vector<Point2i> fingersIJ,
				protob::Hand* protobOneHand);

			/**
			* serialize wrist
			* @param wristXYZ to serialize
			* @param wristIJ to serialize
			* @param protobOneHand to receive
			*/
			void  SerializeHandWrist(std::vector<Vec3f> wristXYZ,
				std::vector<Point2i> wristIJ,
				protob::Hand* protobOneHand);

			/**
			* serialize palmcenter
			* @param palmcenterXYZ to serialize
			* @param palmcenterIJ to serialize
			* @param protobOneHand to receive
			*/
			void  SerializeHandPalmcenter(Vec3f palmcenterXYZ,
				Point2i palmcenterIJ,
				protob::Hand* protobOneHand);

			/**
			* deserialize hand
			* @param hand_protobuffer_string to deserialize
			*/
			protob::Hands DeserializHand(std::string hand_protobuffer_string);

	}
}
