#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
 
#include <gazebo_grasp_plugin/GazeboGraspFix.h>

using gazebo::GazeboGraspFix;


#define DEFAULT_FORCES_ANGLE_TOLERANCE 120
#define DEFAULT_UPDATE_RATE 5
#define DEFAULT_MAX_GRIP_COUNT 10    
#define DEFAULT_RELEASE_TOLERANCE 0.005
#define DEFAULT_DISABLE_COLLISIONS_ON_ATTACH false

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)


GazeboGraspFix::GazeboGraspFix(){
    InitValues();
}

GazeboGraspFix::GazeboGraspFix(physics::ModelPtr _model)
{
    InitValues();
}

GazeboGraspFix::~GazeboGraspFix()
{
    this->update_connection.reset();
    if (this->node) this->node->Fini();
    this->node.reset();
}

void GazeboGraspFix::Init()
{
    this->prevUpdateTime = common::Time::GetWallTime();
}

void GazeboGraspFix::InitValues()
{
#if GAZEBO_MAJOR_VERSION > 2
    gazebo::common::Console::SetQuiet(false);
#endif

    // float timeDiff=0.25;
    // this->releaseTolerance=0.005;
    // this->updateRate = common::Time(0, common::Time::SecToNano(timeDiff));
    this->prevUpdateTime = common::Time::GetWallTime();
    //float graspedSecs=2;
    //this->maxGripCount=floor(graspedSecs/timeDiff);    
    //this->gripCountThreshold=floor(this->maxGripCount/2);
    this->node = transport::NodePtr(new transport::Node());
}


void GazeboGraspFix::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::cout<<"Loading grasp-fix plugin"<<std::endl;
    
    // ++++++++++++ Read parameters and initialize fields  +++++++++++++++
    
    physics::ModelPtr model = _parent;
    this->world = model->GetWorld();
    physics::PhysicsEnginePtr physics_engine = this->world->Physics();
    
    sdf::ElementPtr disableCollisionsOnAttachElem = _sdf->GetElement("disable_collisions_on_attach");
    if (!disableCollisionsOnAttachElem.get()){
        std::cout<<"GazeboGraspFix: Using  "<<DEFAULT_DISABLE_COLLISIONS_ON_ATTACH<<" because no <disable_collisions_on_attach> element specified."<<std::endl;
        this->disableCollisionsOnAttach = DEFAULT_DISABLE_COLLISIONS_ON_ATTACH;
    } else {
        this->disableCollisionsOnAttach = disableCollisionsOnAttachElem->Get<bool>();
        std::cout<<"GazeboGraspFix: Using disable_collisions_on_attach "<<this->disableCollisionsOnAttach<<std::endl;
    }

    sdf::ElementPtr forcesAngleToleranceElem = _sdf->GetElement("forces_angle_tolerance");
    if (!forcesAngleToleranceElem.get()){
        std::cout<<"GazeboGraspFix: Using default tolerance of "<<DEFAULT_FORCES_ANGLE_TOLERANCE<<" because no <forces_angle_tolerance> element specified."<<std::endl;
        this->forcesAngleTolerance = DEFAULT_FORCES_ANGLE_TOLERANCE * M_PI/180;
    } else {
        this->forcesAngleTolerance = forcesAngleToleranceElem->Get<float>() * M_PI/180;
    }

    sdf::ElementPtr updateRateElem = _sdf->GetElement("update_rate");
    double _updateSecs;
    if (!updateRateElem.get()){
        std::cout<<"GazeboGraspFix: Using  "<<DEFAULT_UPDATE_RATE<<" because no <updateRate_tag> element specified."<<std::endl;
        _updateSecs = 1.0 / DEFAULT_UPDATE_RATE;
    } else {
        int _rate = updateRateElem->Get<int>();
        double _updateRate = _rate;
        _updateSecs = 1.0/_updateRate;
        std::cout<<"GazeboGraspFix: Using update rate "<<_rate<<std::endl;
    }
    this->updateRate = common::Time(0, common::Time::SecToNano(_updateSecs));

    sdf::ElementPtr maxGripCountElem = _sdf->GetElement("max_grip_count");
    if (!maxGripCountElem.get()){
        std::cout<<"GazeboGraspFix: Using  "<<DEFAULT_MAX_GRIP_COUNT<<" because no <max_grip_count> element specified."<<std::endl;
        this->maxGripCount = DEFAULT_MAX_GRIP_COUNT;
    } else {
        this->maxGripCount = maxGripCountElem->Get<int>();
        std::cout<<"GazeboGraspFix: Using max_grip_count "<<this->maxGripCount<<std::endl;
    }

    sdf::ElementPtr gripCountThresholdElem = _sdf->GetElement("grip_count_threshold");
    if (!gripCountThresholdElem.get()){
        this->gripCountThreshold=floor(this->maxGripCount/2.0);
        std::cout<<"GazeboGraspFix: Using  "<<this->gripCountThreshold<<" because no <grip_count_threshold> element specified."<<std::endl;
    } else {
        this->gripCountThreshold = gripCountThresholdElem->Get<int>();
        std::cout<<"GazeboGraspFix: Using grip_count_threshold "<<this->gripCountThreshold<<std::endl;
    }

    sdf::ElementPtr releaseToleranceElem = _sdf->GetElement("release_tolerance");
    if (!releaseToleranceElem.get()){
        std::cout<<"GazeboGraspFix: Using  "<<DEFAULT_RELEASE_TOLERANCE<<" because no <release_tolerance> element specified."<<std::endl;
        this->releaseTolerance = DEFAULT_RELEASE_TOLERANCE;
    } else {
        this->releaseTolerance = releaseToleranceElem->Get<float>();
        std::cout<<"GazeboGraspFix: Using release_tolerance "<<this->releaseTolerance<<std::endl;
    }

    // will contain all names of collision entities involved from all arms
    std::vector<std::string> collisionNames;
    
    sdf::ElementPtr armElem=_sdf->GetElement("arm");
    if (!armElem.get())
    {
        gzerr <<"GazeboGraspFix: Cannot load the GazeboGraspFix without any <arm> declarations"<<std::endl;
        return;
    }
    // add all arms:
    for (; armElem != NULL; armElem = armElem->GetNextElement("arm"))
    {
        sdf::ElementPtr armNameElem = armElem->GetElement("arm_name");
        sdf::ElementPtr handLinkElem = armElem->GetElement("palm_link");
        sdf::ElementPtr fingerLinkElem = armElem->GetElement("gripper_link");

        if (!handLinkElem.get() || !fingerLinkElem.get() || !armNameElem.get()) {
            gzerr << "ERROR: GazeboGraspFix: Cannot use a GazeboGraspFix arm because "
                  << "not all of <arm_name>, <palm_link> and <gripper_link> elements specified in URDF/SDF. Skipping."<<std::endl;
            continue;
        }
            
        std::string armName = armNameElem->Get<std::string>();
        std::string palmName = handLinkElem->Get<std::string>();
        
        // collect all finger names:
        std::vector<std::string> fingerLinkNames;
        for (; fingerLinkElem != NULL; fingerLinkElem = fingerLinkElem->GetNextElement("gripper_link"))
        {
            std::string linkName = fingerLinkElem->Get<std::string>();
            fingerLinkNames.push_back(linkName);
        }
            
        // add new gripper
        if (grippers.find(armName)!=grippers.end())
        {
            gzerr<<"GazeboGraspFix: Arm named "<<armName<<" was already added, cannot add it twice."<<std::endl;
        }
        GazeboGraspGripper& gripper = grippers[armName];
        std::map<std::string, physics::CollisionPtr> _collisions;
        if (!gripper.Init(physics_engine, _parent, armName, palmName, fingerLinkNames, disableCollisionsOnAttach, _collisions))
        {
            gzerr<<"GazeboGraspFix: Could not initialize arm "<<armName<<". Skipping."<<std::endl;
            grippers.erase(armName);
            continue;
        }
        // add all the grippers's collision elements
        for (std::map<std::string, physics::CollisionPtr>::iterator collIt = _collisions.begin();
            collIt != _collisions.end(); ++collIt)
        {
            const std::string& collName=collIt->first;
            //physics::CollisionPtr& coll=collIt->second;
            std::map<std::string, std::string>::iterator collIter = this->collisions.find(collName);
            if (collIter != this->collisions.end()) { //this collision was already added before
                gzwarn <<"GazeboGraspFix: Adding Gazebo collision link element "<<collName<<" multiple times, the grasp plugin may not work properly"<<std::endl;
                continue;
            }
            std::cout<<"GazeboGraspFix: Adding collision scoped name "<<collName<<std::endl;
            this->collisions[collName] = armName;
            collisionNames.push_back(collName);
        }
    }

    if (grippers.empty())
    {
        gzerr << "ERROR: GazeboGraspFix: Cannot use a GazeboGraspFix because "
              << "no arms were configured successfully. Plugin will not work."<<std::endl;
        return;
    }

    // ++++++++++++ start up things +++++++++++++++


    this->node->Init(this->world->Name());
    physics::ContactManager * contactManager = physics_engine->GetContactManager();
    contactManager->PublishContacts(); //XXX not sure I need this?

    std::string topic = contactManager->CreateFilter(model->GetScopedName(), collisionNames);
    if (!this->contactSub.get()) {
        std::cout<<"Subscribing contact manager to topic "<<topic<<std::endl;
        bool latching=false;
        this->contactSub = this->node->Subscribe(topic,&GazeboGraspFix::OnContact, this, latching);
    }

    update_connection=event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboGraspFix::OnUpdate, this));
}


class GazeboGraspFix::ObjectContactInfo
{
    public:
     
    // all forces effecting on the object
    std::vector<ignition::math::Vector3<double>> appliedForces;

    // all grippers involved in the process, along with
    // a number counting the number of contact points with the
    // object per gripper
    std::map<std::string, int> grippersInvolved;
    
    // maximum number of contacts of *any one* gripper
    // (any in grippersInvolved)
    int maxGripperContactCnt;
    
    // the gripper for maxGripperContactCnt
    std::string maxContactGripper;
};



bool GazeboGraspFix::isGripperLink(const std::string& linkName, std::string& gripperName) const
{
    for (std::map<std::string, GazeboGraspGripper>::const_iterator it=grippers.begin(); it!=grippers.end(); ++it)
    {
        if (it->second.hasLink(linkName))
        {
            gripperName=it->first;
            return true;
        }
    }
    return false;
}   

std::map<std::string, std::string> GazeboGraspFix::getAttachedObjects() const
{
    std::map<std::string, std::string> ret;
    for (std::map<std::string, GazeboGraspGripper>::const_iterator it=grippers.begin(); it!=grippers.end(); ++it)
    {
        const std::string& gripperName = it->first;
        const GazeboGraspGripper& gripper = it->second;
        if (gripper.isObjectAttached())
        {
            ret[gripper.attachedObject()]=gripperName;
        }
    }
    return ret;
}   





bool GazeboGraspFix::objectAttachedToGripper(const ObjectContactInfo& objContInfo, std::string& attachedToGripper) const
{
    for (std::map<std::string, int>::const_iterator gripInvIt=objContInfo.grippersInvolved.begin();
            gripInvIt != objContInfo.grippersInvolved.end(); ++gripInvIt)
    {
        const std::string& gripperName=gripInvIt->first;
        if (objectAttachedToGripper(gripperName, attachedToGripper))
        {
            return true; 
        }
    }
    return false;
}

bool GazeboGraspFix::objectAttachedToGripper(const std::string& gripperName, std::string& attachedToGripper) const
{
    std::map<std::string, GazeboGraspGripper>::const_iterator gIt=grippers.find(gripperName);
    if (gIt == grippers.end())
    {
        gzerr << "GazeboGraspFix: Inconsistency, gripper "<<gripperName<<" not found in GazeboGraspFix grippers"<<std::endl;
        return false; 
    }
    const GazeboGraspGripper& gripper = gIt->second;
    // std::cout<<"Gripper "<<gripperName<<" is involved in the grasp"<<std::endl;
    if (gripper.isObjectAttached())
    {
        attachedToGripper = gripperName;
        return true;
    }
    return false;
}


/**
 * Helper class to encapsulate a collision information.
 * One contact has two bodies, and only
 * the ones where one of the bodies is a gripper link are considered.
 * Each contact consists of a *list* of forces with their own origin/position each
 * (e.g. when the object and gripper are colliding at several places).
 * The averages of each contact's force vectors along with their origins are
 * *accumulated* in the given Vector3 \e pos and \eforce objects.
 * The number of additions is stored in \e sum.
 * This is to get the average force application over time between link and object.
 * 
 * \author Jennifer Buehler
 */
class GazeboGraspFix::CollidingPoint{
public:
    CollidingPoint(): sum(0) {}
    CollidingPoint(const CollidingPoint& o):
        gripperName(o.gripperName),
        collLink(o.collLink),
        collObj(o.collObj),
        force(o.force),
        pos(o.pos),
        objPos(o.objPos),
        sum(o.sum){}

    // Name of the gripper/arm involved in contact point
    // This is not the specific link, but the name of the
    // whole gripper
    std::string gripperName;

    // the collision        
    physics::CollisionPtr collLink, collObj;

    // average force vector of the colliding point
    ignition::math::Vector3<double> force;

    // position (relative to reference frame of gripper
    // collision surface) where the contact happens on collision surface
    ignition::math::Vector3<double> pos;

    // position (relative to reference frame of *gripper* collision surface)
    // where the object center is located during collision. 
    ignition::math::Vector3<double> objPos;
     
    // sum of force and pose (they are actually summed
    // up from several contact points).
    // Divide both \e force and \e pos by this to obtain average
    int sum;
};



void GazeboGraspFix::OnUpdate() {
    if ((common::Time::GetWallTime() - this->prevUpdateTime) < this->updateRate)
        return;

    // first, copy all contact data into local struct. Don't do the complex grip check (checkGrip)
    // within the mutex, because that slows down OnContact().
    this->mutexContacts.lock();
    std::map<std::string, std::map<std::string, CollidingPoint> > contPoints(this->contacts);
    this->contacts.clear(); // clear so it can be filled anew by OnContact().
    this->mutexContacts.unlock();

    // contPoints now contains CollidingPoint objects for each *object* and *link*.


    // Iterate through all contact points to gather all summed forces
    // (and other useful information) for all the objects (so we have all forces on one object). 
    std::map<std::string, std::map<std::string, CollidingPoint> >::iterator objIt;
    std::map<std::string, ObjectContactInfo> objectContactInfo;

    for (objIt=contPoints.begin(); objIt!=contPoints.end(); ++objIt)
    {
        std::string objName=objIt->first;
        //std::cout<<"Examining object collisions with "<<objName<<std::endl;
            
        // create new entry in accumulated results map and get reference to fill in:
        ObjectContactInfo& objContInfo = objectContactInfo[objName];
       
        // for all links colliding with this object... 
        std::map<std::string, CollidingPoint>::iterator lIt;
        for (lIt=objIt->second.begin(); lIt!=objIt->second.end(); ++lIt){
            std::string linkName=lIt->first;
            CollidingPoint& collP=lIt->second;
            ignition::math::Vector3<double> avgForce=collP.force/collP.sum;
            // std::cout << "Found collision with "<<linkName<<": "<<avgForce.x<<", "<<avgForce.y<<", "<<avgForce.z<<" (avg over "<<collP.sum<<")"<<std::endl;
            objContInfo.appliedForces.push_back(avgForce);
            // insert the gripper (if it doesn't exist yet) and increase contact counter
            int& gContactCnt = objContInfo.grippersInvolved[collP.gripperName];
            gContactCnt++;
            int& _maxGripperContactCnt=objContInfo.maxGripperContactCnt;
            if (gContactCnt > _maxGripperContactCnt)
            {
                _maxGripperContactCnt = gContactCnt;
                objContInfo.maxContactGripper=collP.gripperName;
            }
        }
    }
 
    // ++++++++++++++++++++ Handle Attachment  +++++++++++++++++++++++++++++++

    // collect of all objects which are found to be "gripped" at the current stage.
    // if they are gripped, increase the grip counter. If the grip count exceeds the
    // threshold, attach the object to the gripper which has most contact points with the
    // object.
    std::set<std::string> grippedObjects;    
    for (std::map<std::string, ObjectContactInfo>::iterator ocIt=objectContactInfo.begin();
            ocIt!=objectContactInfo.end(); ++ocIt)
    {
        const std::string& objName=ocIt->first;
        const ObjectContactInfo& objContInfo = ocIt->second;
    
        // std::cout<<"Number applied forces on "<<objName<<": "<<objContInfo.appliedForces.size()<<std::endl;
        
        float minAngleDiff= this->forcesAngleTolerance; //120 * M_PI/180;
        if (!checkGrip(objContInfo.appliedForces, minAngleDiff, 0.3)) 
            continue;
        
        // add to "gripped objects" 
        grippedObjects.insert(objName);
        
        // std::cout<<"Grasp Held: "<<objName<<" grip count: "<<this->gripCounts[objName]<<std::endl;

        int& counts = this->gripCounts[objName];
        if (counts < this->maxGripCount) ++counts;                

        // only need to attach object if the grip count threshold is exceeded        
        if (counts <= this->gripCountThreshold)
            continue;
        
        // std::cout<<"GRIPPING "<<objName<<", grip count "<<counts<<" (threshold "<<this->gripCountThreshold<<")"<<std::endl;
        
        // find out if any of the grippers involved in the grasp is already grasping the object.
        // If there is no such gripper, attach it to the gripper which has most contact points.
        std::string attachedToGripper;
        bool isAttachedToGripper=objectAttachedToGripper(objContInfo, attachedToGripper);
        if (isAttachedToGripper)
        {   // the object is already attached to a gripper, so it does not need to be attached.
            // std::cout << "GazeboGraspFix has found that object "<<gripper.attachedObject()<<" is already attached to gripper "<<gripperName;
            continue;       
        }

        // attach the object to the gripper with most contact counts
        const std::string& graspingGripperName = objContInfo.maxContactGripper;
        std::map<std::string, GazeboGraspGripper>::iterator gIt=grippers.find(graspingGripperName);
        if (gIt == grippers.end())
        {
            gzerr << "GazeboGraspFix: Inconsistency, gripper '"<<graspingGripperName
                  << "' not found in GazeboGraspFix grippers, so cannot do attachment of object " << objName << std::endl;
            continue;
        }
        GazeboGraspGripper& graspingGripper = gIt->second;
                
        if (graspingGripper.isObjectAttached())
        {
           gzerr<<"GazeboGraspFix has found that object "<<
                 graspingGripper.attachedObject()<<" is already attached to gripper "<<
                 graspingGripperName<<", so can't grasp '"<<objName<<"'!"<<std::endl;
            continue;
        }

        std::cout<<"GazeboGraspFix: Attaching "<<objName<<" to gripper "<<graspingGripperName<<"!!!!!!!"<<std::endl;

        // Store the array of contact poses which played part in the grip, sorted by colliding link.
        // Filter out all link names of other grippers, otherwise if the other gripper moves
        // away, this is going to trigger the release condition.
        // XXX this does not consider full support for an object being gripped by two grippers (e.g.
        // one left, one right).
        // this->attachGripContacts[objName]=contPoints[objName];
        const std::map<std::string, CollidingPoint>& contPointsTmp = contPoints[objName];
        std::map<std::string, CollidingPoint>& attGripConts = this->attachGripContacts[objName];
        attGripConts.clear();
        std::map<std::string, CollidingPoint>::const_iterator contPointsIt;
        for (contPointsIt=contPointsTmp.begin(); contPointsIt!=contPointsTmp.end(); ++contPointsIt)
        {
            const std::string& collidingLink = contPointsIt->first;
            const CollidingPoint& collidingPoint = contPointsIt->second;
            // std::cout<<"Checking initial contact with "<<collidingLink<<" and "<<graspingGripperName<<std::endl;
            if (graspingGripper.hasCollisionLink(collidingLink))
            {
                // std::cout<<"Insert initial contact with "<<collidingLink<<std::endl;
                attGripConts[collidingLink] = collidingPoint;
            }
        } 

        if (!graspingGripper.HandleAttach(objName)){
            gzerr<<"GazeboGraspFix: Could not attach object "<<objName<<" to gripper "<<graspingGripperName<<std::endl;
        }
        this->OnAttach(objName, graspingGripperName);
    }  // for all objects



    // ++++++++++++++++++++ Handle Detachment  +++++++++++++++++++++++++++++++
    std::map<std::string, std::string> attachedObjects = getAttachedObjects();

    // now, for all objects that are not currently gripped,
    // decrease grip counter, and possibly release object.
    std::map<std::string, int>::iterator gripCntIt;
    for (gripCntIt = this->gripCounts.begin(); gripCntIt != this->gripCounts.end(); ++gripCntIt){

        const std::string& objName=gripCntIt->first;

        if (grippedObjects.find(objName) != grippedObjects.end())
        {   // this object is one we just detected as "gripped", so no need to check for releasing it...
            // std::cout<<"NOT considering "<<objName<<" for detachment."<<std::endl;
            continue;
        }
        
        // the object does not satisfy "gripped" criteria, so potentially has to be released.

        // std::cout<<"NOT-GRIPPING "<<objName<<", grip count "<<gripCntIt->second<<" (threshold "<<this->gripCountThreshold<<")"<<std::endl;

        if (gripCntIt->second > 0) --(gripCntIt->second);
    
        std::map<std::string, std::string>::iterator attIt = attachedObjects.find(objName);
        bool isAttached = (attIt != attachedObjects.end());
        
        // std::cout<<"is attached: "<<isAttached<<std::endl;

        if (!isAttached || (gripCntIt->second > this->gripCountThreshold)) continue;

        const std::string& graspingGripperName=attIt->second;

        // std::cout<<"Considering "<<objName<<" for detachment."<<std::endl;

        // Object should potentially be detached now.
        // However, this happens too easily when just considering the count, as the fingers
        // in gazebo start wobbling as the arm moves around, and although they are still
        // close to the object, the grip is not detected any more. So to be sure, we will
        // check that the collision point (the place on the link where the contact originally
        // was detected) has not moved too far from where it originally was, relative to the object.

        // get the initial set of CollidingPoints for this object
        // note that it is enough to use the initial contact points, because the object won't
        // have "slipped" after being attached, and the location of the original contact point
        // on the link itself is considered as a fixed point on the link, regardless whether this
        // point is currently still colliding with the object or not. We only want to know whether
        // this fixed point on the link has increased in distance from the corresponding fixed
        // point (where the contact originally happened) on the object.
        std::map<std::string, std::map<std::string, CollidingPoint> >::iterator initCollIt=this->attachGripContacts.find(objName);
        if (initCollIt == this->attachGripContacts.end()) {
            std::cerr<<"ERROR: Consistency: Could not find attachGripContacts for "<<objName<<std::endl;
            continue;
        }

        std::map<std::string, CollidingPoint>& initColls=initCollIt->second;
        int cntRelease=0;
       
        // for all links which have initially been detected to collide: 
        std::map<std::string, CollidingPoint>::iterator pointIt;
        for (pointIt=initColls.begin(); pointIt!=initColls.end(); ++pointIt)
        {
            CollidingPoint& cpInfo=pointIt->second;
            // initial distance from link to contact point (relative to link)
            ignition::math::Vector3<double> relContactPos=cpInfo.pos/cpInfo.sum;
            // initial distance from link to object (relative to link)
            ignition::math::Vector3<double> relObjPos=cpInfo.objPos/cpInfo.sum;
           
            // get current world pose of object 
            ignition::math::Pose3 currObjWorldPose=cpInfo.collObj->GetLink()->WorldPose();

            // get world pose of link
            ignition::math::Pose3 currLinkWorldPose=cpInfo.collLink->GetLink()->WorldPose();

            // Get transform for currLinkWorldPose as matrix
            ignition::math::Matrix4 worldToLink(currLinkWorldPose.Rot());
            worldToLink.Translate(currLinkWorldPose.Pos());

            // Get the transform from collision link to contact point
            ignition::math::Matrix4 linkToContact=ignition::math::Matrix4<double>::Identity;
            linkToContact.Translate(relContactPos);
                    
            // the current world position of the contact point right now is:
            ignition::math::Matrix4<double> _currContactWorldPose=worldToLink*linkToContact;
            ignition::math::Vector3<double> currContactWorldPose=_currContactWorldPose.Translation();

            // the initial contact point location on the link should still correspond
            // to the initial contact point location on the object.

            // initial vector from object center to contact point (relative to link,
            // because relObjPos and relContactPos are from center of link)
            ignition::math::Vector3<double> oldObjDist= relContactPos - relObjPos;
            // the same vector as \e oldObjDist, but calculated by the current world pose
            // of object and the current location of the initial contact location on the link.
            ignition::math::Vector3<double> newObjDist= currContactWorldPose - currObjWorldPose.Pos(); // new distance from contact to object
            
            //std::cout<<"Obj Trans "<<cpInfo.collLink->GetName()<<": "<<relObjPos.x<<", "<<relObjPos.y<<", "<<relObjPos.z<<std::endl;
            //std::cout<<"Cont Trans "<<cpInfo.collLink->GetName()<<": "<<relContactPos.x<<", "<<relContactPos.y<<", "<<relContactPos.z<<std::endl;
        
            // the difference between these vectors should not be too large...
            float diff=fabs(oldObjDist.Length() - newObjDist.Length());
            // std::cout<<"Diff for link "<<cpInfo.collLink->GetName()<<": "<<diff<<std::endl;

            if (diff > releaseTolerance) {
                ++cntRelease;
            }
        }

        if (cntRelease > 0)
        {   // sufficient links did not meet the criteria to be close enough to the object.
            // First, get the grasping gripper:
            std::map<std::string, GazeboGraspGripper>::iterator gggIt = grippers.find(graspingGripperName);
            if (gggIt == grippers.end())
            {
                gzerr << "GazeboGraspFix: Inconsistency: Gazebo gripper '"<<graspingGripperName<<"' not found when checking for detachment" << std::endl;
                continue;
            }
            GazeboGraspGripper& graspingGripper = gggIt->second;
            // Now, detach the object:
            std::cout<<"GazeboGraspFix: Detaching "<<objName<<" from gripper "<<graspingGripperName<<"!!!!!!!"<<std::endl;
            graspingGripper.HandleDetach(objName);
            this->OnDetach(objName,graspingGripperName);
            gripCntIt->second=0;
            this->attachGripContacts.erase(initCollIt);
        }
    }

    this->prevUpdateTime = common::Time::GetWallTime();
}
    
double angularDistance(const ignition::math::Vector3<double>& _v1, const ignition::math::Vector3<double>& _v2) {
    ignition::math::Vector3 v1=_v1;        
    ignition::math::Vector3 v2=_v2;
    v1.Normalize();
    v2.Normalize();
    return acos(v1.Dot(v2));    
}

bool GazeboGraspFix::checkGrip(const std::vector<ignition::math::Vector3<double>>& forces, float minAngleDiff, float lengthRatio){
    if (((lengthRatio > 1) || (lengthRatio < 0)) && (lengthRatio > 1e-04 && (fabs(lengthRatio-1) > 1e-04)))  {
        std::cerr<<"ERROR: checkGrip: always specify a lengthRatio of [0..1]"<<std::endl;
        return false;
    }
    if (minAngleDiff < M_PI_2){
        std::cerr<<"ERROR: checkGrip: min angle must be at least 90 degrees (PI/2)"<<std::endl;
        return false;
    }
    std::vector<ignition::math::Vector3<double>>::const_iterator it1, it2;
    for (it1=forces.begin(); it1!=forces.end(); ++it1){
        ignition::math::Vector3<double> v1=*it1;
        for (it2=it1+1; it2!=forces.end(); ++it2){
            ignition::math::Vector3<double> v2=*it2;
            float l1=v1.Length();
            float l2=v2.Length();
            if ((l1<1e-04) || (l2<1e-04)) continue;
            /*ignition::math::Vector3<double> _v1=v1;
            ignition::math::Vector3<double> _v2=v2;
            _v1/=l1;
            _v2/=l2;
            float angle=acos(_v1.Dot(_v2));*/
            float angle=angularDistance(v1, v2);
            // std::cout<<"Angular distance between v1.len="<<v1.Length()<<" and v2.len="<<v2.Length()<<": "<<angle*180/M_PI<<std::endl;
            if (angle > minAngleDiff) {
                float ratio;
                if (l1>l2) ratio=l2/l1;
                else ratio=l1/l2;
                // std::cout<<"Got angle "<<angle<<", ratio "<<ratio<<std::endl;
                if (ratio >= lengthRatio)
                {
                    // std::cout<<"checkGrip() is true"<<std::endl;
                    return true;
                }
            }            
        }
    }
    return false;
}

void GazeboGraspFix::OnContact(const ConstContactsPtr &_msg)
{
    // std::cout<<"CONTACT! "<<std::endl;//<<_contact<<std::endl;
    // for all contacts...
    for (int i = 0; i < _msg->contact_size(); ++i) {
        physics::CollisionPtr collision1 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->EntityByName(_msg->contact(i).collision1()));
        physics::CollisionPtr collision2 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->EntityByName(_msg->contact(i).collision2()));

        if ((collision1 && !collision1->IsStatic()) && (collision2 && !collision2->IsStatic()))
        {
            std::string name1 = collision1->GetScopedName();
            std::string name2 = collision2->GetScopedName();

            // std::cout<<"OBJ CONTACT! "<<name1<<" / "<<name2<<std::endl;
            int count = _msg->contact(i).position_size();

            // Check to see if the contact arrays all have the same size.
            if ((count != _msg->contact(i).normal_size()) ||
                (count != _msg->contact(i).wrench_size()) ||
                (count != _msg->contact(i).depth_size()))
            {
                gzerr << "GazeboGraspFix: Contact message has invalid array sizes\n"<<std::endl;
                continue;
            }

            std::string collidingObjName, collidingLink, gripperOfCollidingLink;
            physics::CollisionPtr linkCollision;
            physics::CollisionPtr objCollision;

            physics::Contact contact;
            contact = _msg->contact(i);

            if (contact.count<1)
            {
                std::cerr<<"ERROR: GazeboGraspFix: Not enough forces given for contact of ."<<name1<<" / "<<name2<<std::endl;
                continue;
            }

            // all force vectors which are part of this contact
            std::vector<ignition::math::Vector3<double>> force;
            
            // find out which part of the colliding entities is the object, *not* the gripper,
            // and copy all the forces applied to it into the vector 'force'.
            std::map<std::string,std::string>::const_iterator gripperCollIt = this->collisions.find(name2);
            if (gripperCollIt != this->collisions.end())
            {   // collision 1 is the object
                collidingObjName=name1;
                collidingLink=name2;
                linkCollision=collision2;
                objCollision=collision1;
                gripperOfCollidingLink=gripperCollIt->second;
                for (int k=0; k<contact.count; ++k)
                    force.push_back(contact.wrench[k].body1Force);
            }
            else if ((gripperCollIt=this->collisions.find(name1)) != this->collisions.end())
            {   // collision 2 is the object
                collidingObjName=name2;
                collidingLink=name1;
                linkCollision=collision1;
                objCollision=collision2;
                gripperOfCollidingLink=gripperCollIt->second;
                for (int k=0; k<contact.count; ++k)
                    force.push_back(contact.wrench[k].body2Force);
            }
            // std::cout<<"FORCES SIZE: "<< force.size()<< std::endl;
            ignition::math::Vector3<double> avgForce;
            // compute average/sum of the forces applied on the object
            for (int k=0; k<force.size(); ++k){
                avgForce+=force[k];
            }    
            avgForce/=force.size();

            ignition::math::Vector3<double> avgPos;
            // compute center point (average pose) of all the origin positions of the forces appied
            for (int k=0; k<contact.count; ++k) avgPos+=contact.positions[k];
            avgPos/=contact.count;

            // now, get average pose relative to the colliding link
            ignition::math::Pose3 linkWorldPose=linkCollision->GetLink()->WorldPose();

            // To find out the collision point relative to the Link's local coordinate system, first get the Poses as 4x4 matrices
            ignition::math::Matrix4 worldToLink(linkWorldPose.Rot());
            worldToLink.Translate(linkWorldPose.Pos());
            
            ignition::math::Matrix4 worldToContact=ignition::math::Matrix4<double>::Identity;
            //we can assume that the contact has identity rotation because we don't care about its orientation.
            //We could always set another rotation here too.
            worldToContact.Translate(avgPos);

            // now, worldToLink * contactInLocal = worldToContact
            // hence, contactInLocal = worldToLink.Inv * worldToContact
            ignition::math::Matrix4 worldToLinkInv = worldToLink.Inverse();
            ignition::math::Matrix4 contactInLocal = worldToLinkInv * worldToContact;
            ignition::math::Vector3<double> contactPosInLocal = contactInLocal.Translation();
            
            //std::cout<<"---------"<<std::endl;    
            //std::cout<<"CNT in loc: "<<contactPosInLocal.x<<","<<contactPosInLocal.y<<","<<contactPosInLocal.z<<std::endl;

            /*ignition::math::Vector3<double> sDiff=avgPos-linkWorldPose.Pos();
            std::cout<<"SIMPLE trans: "<<sDiff.x<<","<<sDiff.y<<","<<sDiff.z<<std::endl;
            std::cout<<"coll world pose: "<<linkWorldPose.Pos().x<<", "<<linkWorldPose.Pos().y<<", "<<linkWorldPose.Pos().z<<std::endl; 
            std::cout<<"contact avg pose: "<<avgPos.x<<", "<<avgPos.y<<", "<<avgPos.z<<std::endl; 

            ignition::math::Vector3<double> lX=linkWorldPose.rot.XAxis();    
            ignition::math::Vector3<double> lY=linkWorldPose.rot.YAxis();    
            ignition::math::Vector3<double> lZ=linkWorldPose.rot.ZAxis();    
    
            std::cout<<"World ori: "<<linkWorldPose.rot.x<<","<<linkWorldPose.rot.y<<","<<linkWorldPose.rot.z<<","<<linkWorldPose.rot.w<<std::endl;
            std::cout<<"x axis: "<<lX.x<<","<<lX.y<<","<<lX.z<<std::endl;
            std::cout<<"y axis: "<<lY.x<<","<<lY.y<<","<<lY.z<<std::endl;
            std::cout<<"z axis: "<<lZ.x<<","<<lZ.y<<","<<lZ.z<<std::endl;*/

            // now, get the pose of the object and compute it's relative position to the collision surface.
            ignition::math::Pose3 objWorldPose = objCollision->GetLink()->WorldPose();
            ignition::math::Matrix4 worldToObj(objWorldPose.Rot());
            worldToObj.Translate(objWorldPose.Pos());
    
            ignition::math::Matrix4 objInLocal = worldToLinkInv * worldToObj;
            ignition::math::Vector3<double> objPosInLocal = objInLocal.Translation();

            {
                boost::mutex::scoped_lock lock(this->mutexContacts);
                CollidingPoint& p = this->contacts[collidingObjName][collidingLink];  // inserts new entry if doesn't exist
                p.gripperName=gripperOfCollidingLink;
                p.collLink=linkCollision;
                p.collObj=objCollision;
                p.force+=avgForce;
                p.pos+=contactPosInLocal;
                p.objPos+=objPosInLocal;
                p.sum++;
            }
            //std::cout<<"Average force of contact= "<<avgForce.x<<", "<<avgForce.y<<", "<<avgForce.z<<" out of "<<force.size()<<" vectors."<<std::endl;
        }
    }
}


