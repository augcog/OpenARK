#include "ProtobufUtil.h"

using namespace ark;

namespace protob {

	namespace util {
		std::string SerializeHand(std::vector<ark::Hand::Ptr> hands) {
			protob::Hands serializeHands;

			for (auto i = 0; i < hands.size(); i++) {
				protob::Hand* protobHand = serializeHands.add_hands();;
				std::string protobuffer_onehand;
				//serialize finger
				SerializeHandFingers(hands[i]->getFingers(), hands[i]->getFingersIJ(), protobHand);
				//serialize wrist
				SerializeHandWrist(hands[i]->getWrist(), hands[i]->getWristIJ(), protobHand);
				//serialize palmcenter
				SerializeHandPalmcenter(hands[i]->getPalmCenter(), hands[i]->getPalmCenterIJ(), protobHand);
			}

			std::string hand_protobuffer_string;
			serializeHands.SerializeToString(&hand_protobuffer_string);

			return hand_protobuffer_string;
		}

		protob::PointXYZ ConvertVec3fToPointXYZ(Vec3f vec3fXYZ)
		{
			protob::PointXYZ pointXYZ;

			pointXYZ.set_x(vec3fXYZ[0]);
			pointXYZ.set_y(vec3fXYZ[1]);
			pointXYZ.set_z(vec3fXYZ[2]);

			return pointXYZ;
		}

		protob::PointIJ ConvertPoint2iToPointIJ(Point2i point2i)
		{
			protob::PointIJ PointIJ;

			PointIJ.set_i(point2i.x);
			PointIJ.set_j(point2i.y);

			return PointIJ;
		}

		void SerializeHandFingers(std::vector<Vec3f> fingersXYZ, std::vector<Point2i> fingersIJ, protob::Hand* protobOneHand) {

			for (int i = 0; i < fingersXYZ.size(); ++i)
			{
				protob::PointXYZ FingerPointXYZ = ConvertVec3fToPointXYZ(fingersXYZ[i]);
				protob::PointIJ FingerPointIJ = ConvertPoint2iToPointIJ(fingersIJ[i]);
				protob::Point *finger = protobOneHand->add_fingers();

				finger->mutable_pointxyz()->MergeFrom(FingerPointXYZ);
				finger->mutable_pointij()->MergeFrom(FingerPointIJ);
			}
		}

		void  SerializeHandWrist(std::vector<Vec3f> wristXYZ, std::vector<Point2i> wristIJ, protob::Hand* protobOneHand) {

			for (int i = 0; i < wristXYZ.size(); ++i)
			{
				protob::PointXYZ wristPointXYZ = ConvertVec3fToPointXYZ(wristXYZ[i]);
				protob::PointIJ wristPointIJ = ConvertPoint2iToPointIJ(wristIJ[i]);
				protob::Point *wrist = protobOneHand->add_wrist();

				wrist->mutable_pointxyz()->MergeFrom(wristPointXYZ);
				wrist->mutable_pointij()->MergeFrom(wristPointIJ);
			}
		}

		void  SerializeHandPalmcenter(Vec3f palmcenterXYZ, Point2i palmcenterIJ, protob::Hand* protobOneHand) {
			protob::Point palmcenter;
			protob::PointXYZ palmCenterPointXYZ = ConvertVec3fToPointXYZ(palmcenterXYZ);
			protob::PointIJ palmCenterPointIJ = ConvertPoint2iToPointIJ(palmcenterIJ);

			palmcenter.mutable_pointxyz()->MergeFrom(palmCenterPointXYZ);
			palmcenter.mutable_pointij()->MergeFrom(palmCenterPointIJ);
			protobOneHand->mutable_palmcenter()->MergeFrom(palmcenter);
		}

		protob::Hands DeserializHand(std::string hand_protobuffer_string) {
			protob::Hands DeserializeHands;

			DeserializeHands.ParseFromString(hand_protobuffer_string);

			for (auto i = 0; i < DeserializeHands.hands_size(); i++) {

				protob::Hand * hands = DeserializeHands.mutable_hands(i);

				//deserialize fingers
				for (auto j = 0; j < hands->fingers_size(); j++) {
					protob::Point*  fingers = hands->mutable_fingers(j);
					protob::PointXYZ *pointXYZ = fingers->mutable_pointxyz();
					protob::PointIJ *pointIJ = fingers->mutable_pointij();
				}

				//deserialize wrist
				for (auto j = 0; j < hands->wrist_size(); j++) {
					protob::Point* wrist = hands->mutable_wrist(j);
					protob::PointXYZ *pointXYZ = wrist->mutable_pointxyz();
					protob::PointIJ *pointIJ = wrist->mutable_pointij();
				}

				//wrist palmcenter
				protob::Point palmCenter = hands->palmcenter();
				protob::PointXYZ *palmCenterpointXYZ = palmCenter.mutable_pointxyz();
				protob::PointIJ *palmCenterpointIJ = palmCenter.mutable_pointij();
			}

			return DeserializeHands;
		}
	}
}
