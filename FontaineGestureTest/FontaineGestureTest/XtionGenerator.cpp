#include "XtionGenerator.h"
#include <vector>


//#define SKELETON_WIDTH 640
//#define SKELETON_HEIGHT 480
//#define CHANNEL 3

//const int width = 640;
//const int height = 480;


XtionGenerator::XtionGenerator(string name) : Generator(name)
{
	// Initialize attributes

	_xPosition = 0;
	_yPosition = 0;
	_zPosition = 0;

	_pitch = 0.0;
	_roll = 0.0;
	_yaw = 0.0;


	_3DMode = true;
	_3DGroupType = "XTION_SKELETON";

	_2DMode = false;
	_2DGroupType = "XTION_SKELETON";

	_mirrorMode = false;

	_confidenceMin = 0.0;
	_depthMax = INT_MAX;

	_width = 0;
	_height = 0;

	_frequency = 30;

	_useElevation = true;

}

Node* XtionGenerator::clone(string cloneName) const
{
	// A KinectGenerator cannot be cloned, it is better to return the current instance 
	// Another KinectGenerator can be created for openning another device or another file
	return (Node*)this;
}


bool XtionGenerator::start()
{
	activityFlag = true;

	//cout << "Start Generator" << endl;


	openni::Status rc = openni::OpenNI::initialize();
	if(rc != openni::STATUS_OK){ fprintf(stderr, "OpenNI::initialize failed"); return false; }
	rc = device.open(openni::ANY_DEVICE);
	if(rc != openni::STATUS_OK){ fprintf(stderr, "OpenNI::device.open failed"); return false; }	

	nite::Status rc2 = nite::NiTE::initialize();
	if(rc2 != nite::STATUS_OK){ fprintf(stderr, "NiTE::initialize failed"); return false; }
	rc2 = tracker.create(&device);
	if(rc2 != nite::STATUS_OK){ fprintf(stderr, "UserTracker::create failed"); return false; }
	//tracker.setSkeletonSmoothingFactor(0.2);
	// Retrieve one frame
	do{	tracker.readFrame(&frame);	} while(!frame.isValid());

	return true;
}


bool XtionGenerator::generate(map<string,Group3D*>& g3D,map<string,Group2D*>& g2D,map<string,Group1D*>&,map<string,GroupSwitch*>&)
{
	//cout << "xtionID" << _id << endl;

	if(activityFlag){

		nite::Status rc;
		rc = tracker.readFrame(&frame);
		if(!frame.isValid()) return false;

		int videoWidth = frame.getDepthFrame().getWidth();
		int videoHeight = frame.getDepthFrame().getHeight();

		const nite::Array<nite::UserData>& users = frame.getUsers();
		for (int i=0; i<users.getSize(); i++){
			const nite::UserData& user = users[i];

			char buffer [33];
			itoa (_id+(int)user.getId(),buffer,10);
			string myID = buffer;

			if(user.isNew()){
				tracker.startSkeletonTracking(user.getId());

				if(g3D.count(myID)==0){
					// TODO create it
				
					Group3D* g3 = new Group3D(_environment,myID,"XTION_SKELETON");
					Group2D* g2 = new Group2D(_environment,myID,"XTION_SKELETON");

					// Create HOriented point for each joint

					string myJoint;

					itoa (nite::JOINT_HEAD,buffer,10);

					myJoint = buffer;

					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_HEAD);

					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));

					g3->addElement(myJoint,ho1);

					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"HEAD");

					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));

					g2->addElement(myJoint,hv1);


					itoa (nite::JOINT_LEFT_ELBOW,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho2 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_ELBOW);
					ho2->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho2);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_ELBOW");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_LEFT_FOOT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho3 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_FOOT);
					ho3->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho3);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_FOOT");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_LEFT_HAND,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho4 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_HAND);
					ho4->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho4);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_HAND");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_LEFT_HIP,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho5 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_HIP);
					ho5->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho5);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_HIP");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_LEFT_KNEE,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho6 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_KNEE);
					ho6->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho6);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_KNEE");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_LEFT_SHOULDER,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho7 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_SHOULDER);
					ho7->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho7);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_SHOULDER");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_NECK,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho8 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_NECK);
					ho8->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho8);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"NECK");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_RIGHT_ELBOW,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho9 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_ELBOW);
					ho9->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho9);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_ELBOW");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_RIGHT_FOOT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho10 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_FOOT);
					ho10->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho10);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_FOOT");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_RIGHT_HAND,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho11 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_HAND);
					ho11->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho11);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_HAND");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_RIGHT_HIP,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho12 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_HIP);
					ho12->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho12);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_HIP");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_RIGHT_KNEE,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho13 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_KNEE);
					ho13->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho13);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_KNEE");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_RIGHT_SHOULDER,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho14 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_SHOULDER);
					ho14->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho14);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_SHOULDER");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);

					itoa (nite::JOINT_TORSO,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho15 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_TORSO);
					ho15->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho15);
					hv1 = new HOrientedPoint2D(_environment,myJoint,"TORSO");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);


					g3D[myID] = g3;
					g2D[myID] = g2;
				}



				//
			}else if(user.isLost()){
				// TODO delete from map

				cout << "Lost" << endl;
				delete g3D[myID];
				g3D.erase(myID);
				delete g2D[myID];
				g2D.erase(myID);
			}else{

				//cout << "Update User " << user.getId() << endl;

				float coords[2] = {0};
				string myJoint;

				//cout << "U0" << myID.c_str() << endl;

				/*
				if(g3D.count(myID))
				cout << "YES" << endl;
				else
				cout << "NO" << endl;
				*/

				// TODO Update
				Group3D* g3 = g3D.at(myID);
				if(!g3) return false;



				Group2D* g2 = g2D[myID];
				if(!g2) return false;


				{

					itoa (nite::JOINT_HEAD,buffer,10);
					myJoint = buffer;

					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;

					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_HEAD);

					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);

					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));

					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);

					if(!hv1) return false;

					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));

				}

				{
					itoa (nite::JOINT_LEFT_ELBOW,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_LEFT_FOOT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_LEFT_HAND,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_LEFT_HIP,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_LEFT_KNEE,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_LEFT_SHOULDER,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_NECK,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_NECK);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_RIGHT_ELBOW,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_RIGHT_FOOT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_RIGHT_HAND,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_RIGHT_HIP,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_RIGHT_KNEE,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_RIGHT_SHOULDER,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

				{
					itoa (nite::JOINT_TORSO,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = g3->getElementByID(myJoint);
					if(!ho1) return false;
					nite::SkeletonJoint joint = user.getSkeleton().getJoint(nite::JOINT_TORSO);
					tracker.convertJointCoordinatesToDepth(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z,&coords[0],&coords[1]);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(joint.getPosition().x,joint.getPosition().y,joint.getPosition().z),Orientation3D(),joint.getPositionConfidence(),0.0));
					//joint.getOrientation().w;
					HOrientedPoint2D* hv1 = g2->getElementByID(myJoint);
					if(!hv1) return false;
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(coords[0] / videoWidth,coords[1] / videoHeight),0.0,0.0,0.0));
				}

			}
		} // foreach users

	}
	return true;
}

bool XtionGenerator::stop()
{
	cout << "Stop Generator" << endl;

	tracker.destroy();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
	device.close();

	return true;
}

set<string> XtionGenerator::produce() const
{
	set<string> produce;
						produce.insert(LG_ORIENTEDPOINT3D_HEAD);
						produce.insert(LG_ORIENTEDPOINT3D_LEFT_ELBOW);
						produce.insert(LG_ORIENTEDPOINT3D_LEFT_FOOT);
						produce.insert(LG_ORIENTEDPOINT3D_LEFT_HAND);
						produce.insert(LG_ORIENTEDPOINT3D_LEFT_HIP);
						produce.insert(LG_ORIENTEDPOINT3D_LEFT_KNEE);
						produce.insert(LG_ORIENTEDPOINT3D_LEFT_SHOULDER);
						produce.insert(LG_ORIENTEDPOINT3D_NECK);
						produce.insert(LG_ORIENTEDPOINT3D_RIGHT_ELBOW);
						produce.insert(LG_ORIENTEDPOINT3D_RIGHT_FOOT);
						produce.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);
						produce.insert(LG_ORIENTEDPOINT3D_RIGHT_HIP);
						produce.insert(LG_ORIENTEDPOINT3D_RIGHT_KNEE);
						produce.insert(LG_ORIENTEDPOINT3D_RIGHT_SHOULDER);
						produce.insert(LG_ORIENTEDPOINT3D_TORSO);

						produce.insert("HEAD");
						produce.insert("LEFT_ELBOW");
						produce.insert("LEFT_FOOT");
						produce.insert("LEFT_HAND");
						produce.insert("LEFT_HIP");
						produce.insert("LEFT_KNEE");
						produce.insert("LEFT_SHOULDER");
						produce.insert("NECK");
						produce.insert("RIGHT_ELBOW");
						produce.insert("RIGHT_FOOT");
						produce.insert("RIGHT_HAND");
						produce.insert("RIGHT_HIP");
						produce.insert("RIGHT_KNEE");
						produce.insert("RIGHT_SHOULDER");
						produce.insert("TORSO");

	return produce;
}

XtionGenerator::~XtionGenerator(void)
{
}


Point3D XtionGenerator::getRepositionedPoint(Point3D old){
	Point3D newPos;

	newPos.setX(_xPosition+old.getX());
	newPos.setY(_yPosition+(old.getY()*cos(-_roll*M_PI/180))-(old.getZ()*sin(-_roll*M_PI/180)));
	newPos.setZ(_zPosition+(old.getY()*sin(-_roll*M_PI/180))+(old.getZ()*cos(-_roll*M_PI/180)));

	return newPos;
}

void XtionGenerator::useCameraElevation(bool enable){
	_useElevation = enable;
}