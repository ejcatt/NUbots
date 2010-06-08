/*! @file NUKick.cpp
    @brief Implementation of nukick class

    @author Jed Rietveld
 
 Copyright (c) 2010 Jed Rietveld
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NUKick.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "Behaviour/Jobs/MotionJobs/KickJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "NUPlatform/NUSystem.h"

#include "motionconfig.h"

#ifdef USE_WALK
#include "Motion/NUWalk.h"
#endif // USE_WALK

#include "debug.h"
#include "debugverbositynumotion.h"
#include "Tools/Math/General.h"
using namespace mathGeneral;


NUKick::NUKick(NUWalk* walk)
{
    m_walk = walk;
    kinematicModel = new Kinematics();
    IKSys = new Legs();
    poseData.resize(4);
    pose = DO_NOTHING;
    m_kickingLeg = noLeg;

    m_kickIsActive = false;
    m_stateCommandGiven = false;

    m_currentTimestamp = 0;
    m_previousTimestamp = 0;
    loadKickParameters();
}

/*! @brief Destructor for motion module
 */
NUKick::~NUKick()
{
    kill();
    delete IKSys;
    delete kinematicModel;
}

void NUKick::loadKickParameters()
{
    m_defaultMotorGain = 75.0f;                 // Default to 75% gain.
    m_leftLegInitialPose.resize(6,0.0f);
    m_rightLegInitialPose.resize(6,0.0f);
}

bool NUKick::isActive()
{
    return m_kickIsActive;
}

/*! @brief Kills the kick module
 */
void NUKick::kill()
{
    pose = DO_NOTHING;
    m_kickIsActive = false;
    m_stateCommandGiven = false;
}

void NUKick::stop()
{
    m_stateCommandGiven = false;
    // Chose the state that can be transitioned to allowing kick to finish as soon as possible.
    switch(pose)
    {
        case PRE_KICK:
            pose = POST_KICK;
            break;
        case TRANSFER_TO_SUPPORT:
            pose = UNSHIFT_LEG;
            break;
        case LIFT_LEG:
            pose = RETRACT;
            break;
        case SWING:
            pose = RETRACT;
            break;
        default:
            pose = pose;
    }
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUKick::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL || data == NULL)
        return;
    m_data = data;
    m_actions = actions;
    m_previousTimestamp = m_currentTimestamp;
    m_currentTimestamp = data->CurrentTime;
    
    if (m_currentTimestamp - m_previousTimestamp > 200)
    {
        kill();
        return;
    }
    
    doKick();
    #ifdef USE_WALK
    if(pose == PRE_KICK)
        m_walk->process(data, actions);
    #endif // USE_WALK
}

/*! @brief Process a kick job
    @param job the kick job
 */
void NUKick::process(KickJob* job)
{
    debug << "void NUKick::process(KickJob* job)" << endl;
    double time;
    vector<float> kickposition;
    vector<float> kicktarget;
    job->getKick(time, kickposition, kicktarget);

    m_ball_x = kickposition[0];
    m_ball_y = kickposition[1];

    m_target_x = kicktarget[0];
    m_target_y = kicktarget[1];
    m_target_timestamp = time;
    kickToPoint(kickposition, kicktarget);
}


void NUKick::kickToPoint(const vector<float>& position, const vector<float>& target)
{
    // Evaluate the kicking target to start a new kick, or change a kick in progress.
	m_ball_x = position[0];
	m_ball_y = position[1];
	
	m_target_x = target[0];
	m_target_y = target[1];
	m_target_timestamp = nusystem->getTime();

	switch(pose)
	{
		case DO_NOTHING:
		{
			debug << "Choosing leg." << endl;
			chooseLeg();
			return;
		}

		case USE_LEFT_LEG:
		{	
			debug << "Left leg chosen. Lifting left leg." << endl;
			if(chooseLeg())
				return;
                        //liftLeg();
			return;
		}

		case USE_RIGHT_LEG:
		{
			debug << "Right leg chosen. Lifting right leg." << endl;
			if(chooseLeg())
				return;
			liftLeg();
			return;
		}

		case LIFT_LEG:
		{
			if(chooseLeg())
				return;
                        //setLeg();
			return;
		}
		/*
		case ADJUST_YAW: //none of this: yet!
		{
			if(chooseLeg())
				return;
			if(adjustYaw())
				return;
			setLeg();
			return;
		}
		*/
		case SET_LEG:
		{
			if(chooseLeg())
				return;
			//if(adjustYaw()) //not yet
			//	return;
			if(setLeg())
				return;
			poiseLeg();
			return;
		}
		
		case POISE_LEG:
		{
			if(chooseLeg())
				return;
			//if(adjustYaw()) //not yet
			//	return;
			if(setLeg())
				return;
			swing();
                        break;
		}
		
		case SWING:
		{
                        //retract();
                        break;
		}

		case RETRACT:
		{
                        break;
		}
	
		case RESET:
		{
                    break;
		}

		case NO_KICK:
		{
                    break;
		}
	}
}

void NUKick::doKick()
{
    bool done = false;
    vector<float> pos (6, 0);
    vector<float> vel (6, 0);
    vector<float> gain (6, m_defaultMotorGain);

    vector<float> LeftLegTheta(6,0);
    vector<float> RightLegTheta(6,0);

    m_data->getJointPositions(NUSensorsData::LeftLegJoints, LeftLegTheta);
    m_data->getJointPositions(NUSensorsData::RightLegJoints, RightLegTheta);

	IKSys->inputLeft(LeftLegTheta);
	IKSys->inputRight(RightLegTheta);

        debug << "void NUKick::doKick()" << endl;
	switch(pose)
	{
		case DO_NOTHING:
		{
                        debug << "Doing Nothing." << endl;
			break;
		}
                case PRE_KICK:
                {
                    debug << "Doing Pre-Kick..." << endl;
                    done = doPreKick();
                    if(done)
                    {
                        cout << "Pre kick complete!" << endl;
                        debug << "Pre kick complete!" << endl;
                        pose = TRANSFER_TO_SUPPORT;
                    }
                    break;
                }
                case POST_KICK:
                {
                    debug << "Doing Post-Kick..." << endl;
                    done = doPostKick();
                    if(done)
                    {
                        cout << "Post kick complete!" << endl;
                        debug << "Post kick complete!" << endl;
                        pose = DO_NOTHING;
                    }
                    break;
                }
                case TRANSFER_TO_SUPPORT:
		{
                        debug << "Shifting Weight..." << endl;
                        legId_t supportLeg = noLeg;
                        if(m_kickingLeg == rightLeg)
                            supportLeg = leftLeg;
                        else if(m_kickingLeg == leftLeg)
                            supportLeg = rightLeg;
                        else
                        {
                            pose = POST_KICK;
                            break;
                        }
                        // Shift the weight of the robot to the support leg.
                        done = ShiftWeightToFoot(supportLeg,1.0f,0.005);
                        if(done)
                        {
                            //m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            //m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            cout << "Weight now on support foot!" << endl;
                            debug << "Weight now on support foot!" << endl;
                            pose = LIFT_LEG;
                        }
                        /*
			IKSys->useLeftLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                    */
			break;
		}

		case USE_RIGHT_LEG:
		{
                        debug << "Adding joint positions for right kick..." << endl;
			IKSys->useRightLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case LIFT_LEG:
		{
                        debug << "Lifting Leg..." << endl;
                        done = LiftKickingLeg(m_kickingLeg);
                        if(done)
                        {
                            m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            cout << "Leg is now lifted!" << endl;
                            debug << "Leg is now lifted!" << endl;
                            pose = POISE_LEG;
                        }
                        /*
			IKSys->liftLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                        */
			break;
		}

		case ADJUST_YAW:
		{
                        debug << "Adding joint positions to adjust yaw..." << endl;
			IKSys->adjustYaw(poseData[0]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			break;
		}

		case SET_LEG:
		{
                        debug << "Adding joint positions to set leg..." << endl;
			IKSys->setLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case POISE_LEG:
		{
                        debug << "Poising leg..." << endl;
                        done = true;
                        if(done)
                        {
                            cout << "Leg is now poised!" << endl;
                            debug << "Leg is now poised!" << endl;
                            pose = SWING;
                        }
                        /*
			IKSys->setLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                        */
			break;
		}

		case SWING:
		{
                        debug << "Swinging Leg..." << endl;
                        done = SwingLegForward(m_kickingLeg, 0.01);
                        if(done)
                        {
                            cout << "Swing completed!" << endl;
                            debug << "Swing completed!" << endl;
                            pose = RETRACT;
                        }
                        /*
                        debug << "Adding joint positions for swing..." << endl;
			IKSys->moveLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                        */
			break;
		}
                case RETRACT:
                    {
                        debug << "Retracting Leg..." << endl;
                        //done = RetractLeg(m_kickingLeg);
                        done = LiftKickingLeg(m_kickingLeg);
                        if(done)
                        {
                            cout << "Leg Retracted!" << endl;
                            debug << "Leg Retracted!" << endl;
                            //m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            //m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            pose = REALIGN_LEGS;
                        }
                        break;
                    }
                case REALIGN_LEGS:
                    {
                        debug << "Lowering Leg..." << endl;
                        done = LowerLeg(m_kickingLeg);
                        if(done)
                        {
                            cout << "Legs Aligned!" << endl;
                            debug << "Legs Aligned!" << endl;
                            //m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            //m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            pose = UNSHIFT_LEG;
                        }
                        break;
                    }
                case UNSHIFT_LEG:
                {
                        debug << "Unshifting Weight..." << endl;
                        done = this->ShiftWeightToFoot(m_kickingLeg,0.5f,0.005);
                        if(done)
                        {
                            debug << "Weight Unshifted!" << endl;
                            //m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            //m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            pose = POST_KICK;
                        }
                        break;
                }

                case RESET:
		{
                        debug << "Reset" << endl;
                        /*
                        debug << "Adding joint positions for reset..." << endl;
			IKSys->reset();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                        */
			break;
		}

		case NO_KICK:
		{
                        debug << "No kick." << endl;
			break;
		}

		default:
		{	
                        debug << "unknown pose." << endl;
			pose = NO_KICK;
			break;
		}
	}
}

bool NUKick::doPreKick()
{
    debug << "Pre - Kick" << endl;

#ifdef USE_WALK
    if(m_walk)
    {
        debug << "Walk detected - stopping robot..." << endl;
        const float maxStoppedVelocitySum = 0.4f;
        m_walk->process(new WalkJob(0,0,0));
        vector<float> speed;
        m_walk->getCurrentSpeed(speed);

        if(speed.size() < 3) return false;

        bool walkStopped = allZeros(speed);

        vector<float>jointVelocities;
        float jointVelocitySum = 0.0f;
        if(m_data->getJointVelocities(NUSensorsData::BodyJoints, jointVelocities))
        {
            for (int i = 0; i < jointVelocities.size(); i++)
            {
                jointVelocitySum += fabs(jointVelocities[i]);
            }
        }
        debug << "jointVelocitySum = " << jointVelocitySum << endl;
        debug << "Walk stopped = " << walkStopped << endl;
        bool robotStopped = walkStopped && (jointVelocitySum < maxStoppedVelocitySum);
        debug << "Robot stopped = " << robotStopped << endl;
        if(!robotStopped)
            return false;
    }
#endif // USE_WALK
    bool validData = true;
    vector<float>leftJoints;
    vector<float>rightJoints;
    validData = validData && m_data->getJointPositions(NUSensorsData::LeftLegJoints,leftJoints);
    validData = validData && m_data->getJointPositions(NUSensorsData::RightLegJoints,rightJoints);
    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);

    vector<float> leftArmJoints;
    vector<float> rightArmJoints;
    validData = validData && m_data->getJointPositions(NUSensorsData::LeftArmJoints,leftArmJoints);
    validData = validData && m_data->getJointPositions(NUSensorsData::RightArmJoints,rightArmJoints);

    if(!m_stateCommandGiven && validData)
    {
        debug << "Moving to Initial Position" << endl;
    #ifdef USE_WALK
        m_walk->kill();
    #endif // USE_WALK
        vector<float> armpos (4, 0.0f);
        armpos[1] = PI/2.0f;
        float maxSpeed = 0.7;
        double moveTime = nusystem->getTime() + 1000;
        MoveLimbToPositionWithSpeed(NUActionatorsData::LeftLegJoints, leftJoints, m_leftLegInitialPose, maxSpeed , 75.0);
        MoveLimbToPositionWithSpeed(NUActionatorsData::RightLegJoints, rightJoints, m_rightLegInitialPose, maxSpeed , 75.0);
        armpos[0] = PI/8.0;
        armpos[3] = -PI/2.0f;
        MoveLimbToPositionWithSpeed(NUActionatorsData::LeftArmJoints, leftArmJoints, armpos, maxSpeed , 40.0);
        armpos[0] = -PI/8.0;
        armpos[3] = PI/2.0f;
        MoveLimbToPositionWithSpeed(NUActionatorsData::RightArmJoints, rightArmJoints, armpos, maxSpeed , 40.0);
        m_stateCommandGiven = true;
    }
    if(validData)
    {
        if(allEqual(m_leftLegInitialPose, leftJoints, 0.05f) && allEqual(m_leftLegInitialPose, rightJoints, 0.05f))
        {
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}

bool NUKick::doPostKick()
{
    m_kickIsActive = false;
    return true;
}

bool NUKick::ShiftWeightToFoot(legId_t targetLeg, float targetWeightPercentage, float speed)
{
    const float maxShiftSpeed = speed * SpeedMultiplier();
    const float reqiredAccuracy = 0.05;
    const float propGain = 0.05f;
    bool validData = true;
    bool leftContact, rightContact;
    validData = validData && m_data->getFootContact(NUSensorsData::LeftFoot, leftContact);
    validData = validData && m_data->getFootContact(NUSensorsData::RightFoot, rightContact);
    if(!validData) return false;
    float lcopx(0.0f),lcopy(0.0f),rcopx(0.0f),rcopy(0.0f);
    float lforce(0.0f),rforce(0.0f);


    float leftImpactTime, rightImpactTime;
    bool recentLeftImpact = m_data->footImpact(NUSensorsData::LeftFoot,leftImpactTime);
    bool recentRightImpact = m_data->footImpact(NUSensorsData::RightFoot,rightImpactTime);

    if(leftContact)
    {
        validData = validData && m_data->getFootCoP(NUSensorsData::LeftFoot,lcopx,lcopy);
        validData = validData && m_data->getFootForce(NUSensorsData::LeftFoot,lforce);
    }
    if(rightContact)
    {
        validData = validData && m_data->getFootCoP(NUSensorsData::RightFoot,rcopx,rcopy);
        validData = validData && m_data->getFootForce(NUSensorsData::RightFoot,rforce);
    }


    vector<float>leftJoints;
    vector<float>rightJoints;
    validData = validData && m_data->getJointTargets(NUSensorsData::LeftLegJoints,leftJoints);
    validData = validData && m_data->getJointTargets(NUSensorsData::RightLegJoints,rightJoints);
    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);


    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, m_defaultMotorGain);
        float weightPercentage;
        float weightError = 0;
        bool recentImpact;
        float newHipPos;
        if(targetLeg == rightLeg)
        {
            weightPercentage = rforce / (rforce + lforce);
            weightError = targetWeightPercentage - weightPercentage;
            recentImpact = recentRightImpact;
            recentImpact = m_data->CurrentTime - rightImpactTime < 5*TimeBetweenFrames();
            newHipPos = rightJoints[0] + crop(weightError*propGain, -maxShiftSpeed, maxShiftSpeed);
        }
        else if(targetLeg == leftLeg)
        {
            weightPercentage = lforce / (rforce + lforce);
            weightError = weightPercentage - targetWeightPercentage;
            recentImpact = recentLeftImpact;
            recentImpact = m_data->CurrentTime - leftImpactTime < 5*TimeBetweenFrames();
            newHipPos = leftJoints[0] + crop(weightError*propGain, -maxShiftSpeed, maxShiftSpeed);
        }
        if(fabs(weightError) > reqiredAccuracy)
        {
            leftJoints[0] = newHipPos;
            rightJoints[0] = newHipPos;
            leftJoints[4] = -newHipPos;
            rightJoints[4] = -newHipPos;
        }
        else if(!recentImpact)
        {
            return true;
        }
        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftJoints, vel, gain);
        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightJoints, vel, gain);
    }
    return false;
}

bool NUKick::LiftKickingLeg(legId_t kickingLeg)
{
    bool validData = true;

    NUSensorsData::foot_id_t s_supportFoot;
    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_supportLeg;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUActionatorsData::bodypart_id_t a_supportLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    if(kickingLeg == rightLeg)
    {
        s_supportFoot = NUSensorsData::LeftFoot;
        s_kickingFoot = NUSensorsData::RightFoot;
        s_supportLeg = NUSensorsData::LeftLegJoints;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_supportLeg = NUActionatorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
    }
    else if(kickingLeg == leftLeg)
    {
        s_supportFoot = NUSensorsData::RightFoot;
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_supportLeg = NUSensorsData::RightLegJoints;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_supportLeg = NUActionatorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
    }
    else return true;

    bool supportContact, kickContact;
    validData = validData && m_data->getFootContact(s_supportFoot, supportContact);
    validData = validData && m_data->getFootContact(s_kickingFoot, kickContact);
    if(!validData) return false;
    float copx(0.0f),copy(0.0f);
    float force(0.0f);

    if(supportContact)
    {
        validData = validData && m_data->getFootCoP(s_supportFoot,copx,copy);
        validData = validData && m_data->getFootForce(s_supportFoot,force);
    }

    vector<float>supportLegJoints;
    vector<float>kickLegJoints;
    vector<float>kickLegPositions;
    validData = validData && m_data->getJointTargets(s_supportLeg,supportLegJoints);
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickLegJoints);
    validData = validData && m_data->getJointPositions(s_kickingLeg,kickLegPositions);
    validData = validData && (supportLegJoints.size() >= 6) && (kickLegJoints.size() >= 6);

    vector<float>kickLegTargets(kickLegJoints);

    kickLegTargets[3] = 1.0;
    kickLegTargets[1] = -kickLegTargets[3] / 2.0f;
    kickLegTargets[5] = -kickLegTargets[3] / 2.0f;

    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, m_defaultMotorGain);

        // Balance using support leg
        BalanceCoP(supportLegJoints, copx, copy);

        if(!m_stateCommandGiven)
        {
            MoveLimbToPositionWithSpeed(a_kickingLeg, kickLegPositions, kickLegTargets, 0.7 , 75.0);
            m_stateCommandGiven = true;
        }
/*
        // Keep kicking leg paralled in a lengthways orientation.
        kickLegJoints[0] = supportLegJoints[0];
        kickLegJoints[4] = supportLegJoints[4];

        if(kickLegJoints[3] < 2.0)
            kickLegJoints[1] -= liftSpeed;
        else
            kickLegJoints[1] += 0.5*liftSpeed;
        kickLegJoints[3] += 2*liftSpeed;
        kickLegJoints[5] -= liftSpeed;
        if(kickLegJoints[1] > 0.35)
        {
            return true;
        }

        m_actions->addJointPositions(a_supportLeg, nusystem->getTime(), supportLegJoints, vel, gain);
        m_actions->addJointPositions(a_kickingLeg, nusystem->getTime(), kickLegJoints, vel, gain);
        */
        if(allEqual(kickLegTargets, kickLegPositions, 0.05f))
        {
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}

double NUKick::TimeBetweenFrames()
{
    return (m_currentTimestamp - m_previousTimestamp);
}

float NUKick::perSec2perFrame(float value)
{
    return value * (TimeBetweenFrames() / 1000.0);
}

float NUKick::SpeedMultiplier()
{
    return TimeBetweenFrames() / 20.0f;
}

float NUKick::GainMultiplier()
{
    return TimeBetweenFrames() / 20.0f;
}

bool NUKick::IsPastTime(float time){
    return (m_data->CurrentTime > time);
}

void NUKick::BalanceCoP(vector<float>& jointAngles, float CoPx, float CoPy)
{
    // Linear controller to centre CoP
    const float gainx = 0.006 * GainMultiplier();
    const float gainy = 0.006 * GainMultiplier();

    // Vertical correction
    // Ankle Roll
    jointAngles[4] -= gainy * asin(CoPy / 35.0);
    // Hip Roll - Reverse of pitch to maintain vertical torso.
    jointAngles[0] = -jointAngles[4];

    // Horizontal correction
    jointAngles[5] += gainx * asin(CoPx / 35.0f);
    // Hip Pitch - Reverse of pitch to maintain vertical torso.
    jointAngles[2] = 0.5*jointAngles[5];
    return;
}

void NUKick::FlattenFoot(vector<float>& jointAngles)
{
    jointAngles[5] = -(jointAngles[1] + jointAngles[3]);
    return;
}

void NUKick::MaintainSwingHeight(legId_t supportLeg, vector<float>& supportLegJoints, legId_t swingLeg, vector<float>& swingLegJoints, float swingHeight)
{
    return;
}

bool NUKick::SwingLegForward(legId_t kickingLeg, float speed)
{
    float swingSpeed = speed * SpeedMultiplier();
    bool validData = true;

    NUSensorsData::foot_id_t s_supportFoot;
    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_supportLeg;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUActionatorsData::bodypart_id_t a_supportLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    if(kickingLeg == rightLeg)
    {
        s_supportFoot = NUSensorsData::LeftFoot;
        s_kickingFoot = NUSensorsData::RightFoot;
        s_supportLeg = NUSensorsData::LeftLegJoints;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_supportLeg = NUActionatorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
    }
    else if(kickingLeg == leftLeg)
    {
        s_supportFoot = NUSensorsData::RightFoot;
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_supportLeg = NUSensorsData::RightLegJoints;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_supportLeg = NUActionatorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
    }
    else return true;


    bool supportFootContact;
    validData = validData && m_data->getFootContact(s_supportFoot, supportFootContact);

    if(!validData) return false;
    float copx(0.0f),copy(0.0f);
    float force(0.0f);

    if(supportFootContact)
    {
        validData = validData && m_data->getFootCoP(s_supportFoot,copx,copy);
        validData = validData && m_data->getFootForce(s_supportFoot,force);
    }

    vector<float>supportLegJoints;
    vector<float>kickingLegJoints;
    validData = validData && m_data->getJointTargets(s_supportLeg,supportLegJoints);
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickingLegJoints);
    validData = validData && (supportLegJoints.size() >= 6) && (kickingLegJoints.size() >= 6);

    const float targetHipPitch = -1.2;
    const float targetKneePitch = 0.6f;

    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, 100);

//        if(kickingLeg == rightLeg)
//        {
            float hipDiff = targetHipPitch - kickingLegJoints[1];
            float kneeDiff = targetKneePitch - kickingLegJoints[3];

            if((hipDiff > 0) && (kneeDiff > 0)) return true;

            float hipFramesRequired = fabs(hipDiff) / swingSpeed;
            float kneeFramesRequired = fabs(kneeDiff) / swingSpeed;

            float slowestJointSamplesLeft = 0.0f;
            if(hipFramesRequired > slowestJointSamplesLeft) slowestJointSamplesLeft = hipFramesRequired;
            if(kneeFramesRequired > slowestJointSamplesLeft) slowestJointSamplesLeft = kneeFramesRequired;

            debug << "swingSpeed = " << swingSpeed << endl;
            debug << "hipDiff = " << hipDiff << endl;
            debug << "kneeDiff = " << kneeDiff << endl;
            debug << "hipFramesRequired = " << hipFramesRequired << endl;
            debug << "kneeFramesRequired = " << kneeFramesRequired << endl;
            debug << "slowestJointSamplesLeft = " << slowestJointSamplesLeft << endl;

            BalanceCoP(supportLegJoints, copx, copy);
            kickingLegJoints[0] = supportLegJoints[0];
            kickingLegJoints[4] = supportLegJoints[4];

            if(slowestJointSamplesLeft - hipFramesRequired < 1.0)
            {
                kickingLegJoints[1] -= swingSpeed;
                debug << "moving Hip = " << kickingLegJoints[1] << endl;
            }
            if(slowestJointSamplesLeft - kneeFramesRequired < 1.0)
            {
                kickingLegJoints[3] -= swingSpeed;
                debug << "moving Knee = " << kickingLegJoints[3] << endl;
            }
            FlattenFoot(kickingLegJoints);
        m_actions->addJointPositions(a_supportLeg, nusystem->getTime(), supportLegJoints, vel, gain);
        m_actions->addJointPositions(a_kickingLeg, nusystem->getTime(), kickingLegJoints, vel, gain);
    }
    return false;
}

bool NUKick::LowerLeg(legId_t kickingLeg)
{
    const float maxSpeed = 0.01 * SpeedMultiplier();
    const float pGain = 0.1;
    bool validData = true;
    bool leftContact, rightContact;
    validData = validData && m_data->getFootContact(NUSensorsData::LeftFoot, leftContact);
    validData = validData && m_data->getFootContact(NUSensorsData::RightFoot, rightContact);
    if(!validData) return false;
    float lcopx(0.0f),lcopy(0.0f),rcopx(0.0f),rcopy(0.0f);
    float lforce(0.0f),rforce(0.0f);

    if(leftContact)
    {
        validData = validData && m_data->getFootCoP(NUSensorsData::LeftFoot,lcopx,lcopy);
        validData = validData && m_data->getFootForce(NUSensorsData::LeftFoot,lforce);
    }
    if(rightContact)
    {
        validData = validData && m_data->getFootCoP(NUSensorsData::RightFoot,rcopx,rcopy);
        validData = validData && m_data->getFootForce(NUSensorsData::RightFoot,rforce);
    }

    vector<float>leftJoints;
    vector<float>rightJoints;
    validData = validData && m_data->getJointTargets(NUSensorsData::LeftLegJoints,leftJoints);
    validData = validData && m_data->getJointTargets(NUSensorsData::RightLegJoints,rightJoints);
    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);

    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, m_defaultMotorGain);

        if(kickingLeg == rightLeg)
        {
            BalanceCoP(leftJoints, lcopx, lcopy);
            rightJoints[0] = leftJoints[0];
            rightJoints[4] = leftJoints[4];

            const float targetHipPitch = leftJoints[1];
            const float targetKneePitch = leftJoints[3];

            float hipDiff = targetHipPitch - rightJoints[1];
            float kneeDiff = targetKneePitch - rightJoints[3];
            if((fabs(hipDiff) + fabs(kneeDiff)) < 0.1) return true;

            rightJoints[1] += crop(pGain*hipDiff,-maxSpeed,maxSpeed);
            rightJoints[3] += crop(pGain*kneeDiff,-maxSpeed,maxSpeed);
            FlattenFoot(rightJoints);
        }
        else if(kickingLeg == leftLeg)
        {
            BalanceCoP(rightJoints, lcopx, lcopy);
            leftJoints[0] = rightJoints[0];
            leftJoints[4] = rightJoints[4];

            const float targetHipPitch = rightJoints[1];
            const float targetKneePitch = rightJoints[3];

            float hipDiff = targetHipPitch - leftJoints[1];
            float kneeDiff = targetKneePitch - leftJoints[3];
            if((fabs(hipDiff) + fabs(kneeDiff)) < 0.1) return true;

            leftJoints[1] += crop(pGain*hipDiff,-maxSpeed,maxSpeed);
            leftJoints[3] += crop(pGain*kneeDiff,-maxSpeed,maxSpeed);
            FlattenFoot(leftJoints);
        }
        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftJoints, vel, gain);
        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightJoints, vel, gain);
    }
    return false;
}

bool NUKick::chooseLeg()
{
	//currently must be at zero position
	double theta = atan2(m_target_y-m_ball_y, m_target_x-m_ball_x);
	
	//approximate, assume robot torso moves negligible amount and only rotates
	double xtrans = m_ball_x*cos(theta) + m_ball_y*sin(theta);
	double ytrans = m_ball_y*cos(theta) - m_ball_x*sin(theta);
        debug << "bool NUKick::chooseLeg()" << endl;
        debug << "theta - " << theta << endl;
        debug << "theta - " << theta << endl;
        debug << "xtrans - " << xtrans << endl;
        debug << "ytrans - " << ytrans << endl;

        if(!(xtrans>(3+FL)))
	{
		pose = NO_KICK;
                m_kickingLeg = noLeg;
                debug << "Unable to kick. Ball too close!" << endl;
		return true;
	}

	if(ytrans>(HZ+(3*theta/(PI/4))))
	{
            debug << "ball on right..." << endl;
            float KickabilityVariable = 10.0f;
            //float KickabilityVariable = (pow((ytrans-5-(3*theta/(PI/4))),2)+pow(xtrans,2));
            debug << "KickabilityVariable " << KickabilityVariable << endl;
            if(KickabilityVariable<36)
            {
                    //if(pose==USE_LEFT_LEG)
                    if(m_kickingLeg == rightLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                            return false;
                    m_kickingLeg = rightLeg;
                    pose = PRE_KICK;
                    poseData[0] = 0;
                    poseData[0] = 0;
                    poseData[0] = 0;
                    poseData[0] = 500;
                    m_kickIsActive = true;
                    debug << "Right leg chosen." << endl;
                    return true;
            }
            else
            {
                    pose = NO_KICK;
                    m_kickingLeg = noLeg;
                    debug << "Unable to kick with right leg." << endl;
                    return true;
            }
	}
	else
	{
                debug << "ball on left..." << endl;
		if(ytrans<(-HZ-(3*theta/(PI/4))))
		{
                    float KickabilityVariable = 10.0f;
                    //float KickabilityVariable = (pow((ytrans-5-(3*theta/(PI/4))),2)+pow(xtrans,2));
                    debug << "KickabilityVariable " << KickabilityVariable << endl;
                        if(KickabilityVariable<36)
			{
                                //if(pose==USE_RIGHT_LEG)
                                if(m_kickingLeg == leftLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
					return false;
                                m_kickingLeg = leftLeg;
                                pose = PRE_KICK;
				poseData[0] = 0;
				poseData[0] = 0;
				poseData[0] = 0;
				poseData[0] = 500;
                                m_kickIsActive = true;
                                debug << "Left leg chosen." << endl;
				return true;
			}
			else
			{
                                debug << "Unable to kick with left leg." << endl;
                                m_kickingLeg = noLeg;
				pose = NO_KICK;
				return true;
			}
		}
		else
		{
			pose = NO_KICK;
                        m_kickingLeg = noLeg;
                        debug << "Unable to kick with any leg." << endl;
			return true;
		}
	}
}

bool NUKick::liftLeg()
{
	if(IKSys->getLeftPosition()[2]-IKSys->getRightPosition()[2]<-2.5||IKSys->getLeftPosition()[2]-IKSys->getRightPosition()[2]>2.5)
	{
		return false;
	}
	else
	{
                //pose = LIFT_LEG;
		poseData[0] = 0.0;
		poseData[1] = 0.0;
		poseData[2] = 0.0;
		poseData[3] = 500;
	}
}

bool NUKick::adjustYaw()
{

	double theta = atan2(m_target_y-m_ball_y, m_target_x-m_ball_x);
	
	if(theta>0.0873)
	{
		pose = ADJUST_YAW;
		poseData[0] = theta;
		poseData[1] = 0.0;
		poseData[2] = 0.0;
		poseData[3] = 500;
		return true;
	}	
	return false;
}

bool NUKick::setLeg()
{
	double x;
	double y;
	double z;
	bool recal;
	if(IKSys->getLegInUse()==LEFT)
	{
		x = m_ball_x - FL - 3;
		y = m_ball_y;
		z = 3;

		if(x-IKSys->getLeftPosition()[0]<-0.5||x-IKSys->getLeftPosition()[0]>0.5)
		{
			recal = true;
		}
		if(y-IKSys->getLeftPosition()[1]<-0.5||y-IKSys->getLeftPosition()[1]>0.5)
		{
			recal = true;
		}
		if(z-(IKSys->getLeftPosition()[2]-IKSys->getRightPosition()[2])<-0.5||z-(IKSys->getLeftPosition()[2]-IKSys->getRightPosition()[2])>0.5)
		{
			recal = true;
		}
	}
	else
	{
		x = m_ball_x - FL - 3;
		y = m_ball_y;
		z = 3;

		if(x-IKSys->getRightPosition()[0]<-0.5||x-IKSys->getRightPosition()[0]>0.5)
		{
			recal = true;
		}
		if(y-IKSys->getRightPosition()[1]<-0.5||y-IKSys->getRightPosition()[1]>0.5)
		{
			recal = true;
		}
		if(z-(IKSys->getRightPosition()[2]-IKSys->getLeftPosition()[2])<-0.5||z-(IKSys->getRightPosition()[2]-IKSys->getLeftPosition()[2])>0.5)
		{
			recal = true;
		}
	}

	if(recal)
	{
		pose = SET_LEG;
		poseData[0] = x;
		poseData[1] = y;
		poseData[2] = z;
		poseData[3] = 500;
		
	}	
}

bool NUKick::poiseLeg()
{
	double x;
	double y;
	double z;
	
	pose = POISE_LEG;
	poseData[0] = m_ball_x - FL - 8;
	poseData[1] = m_ball_y;
	poseData[2] = 6.8;
	poseData[3] = 500;
	
	return false;
	
}

bool NUKick::swing()
{

	double x;
	double y;
	double z;
	
	pose = SWING;
	poseData[0] = 18;
	poseData[1] = 0;
	poseData[2] = 1.9;
	poseData[3] = 500;
	IKSys->setLeg(13, 10.6, 8.7);

	return false;

}

bool NUKick::retract()
{
	pose = RETRACT;
	return false;
}

void NUKick::MoveLimbToPositionWithSpeed(NUActionatorsData::bodypart_id_t limbId, vector<float> currentPosition, vector<float> targetPosition, float maxSpeed , float gain)
{
    static const float movespeed = maxSpeed;
    vector<float> velocity(m_actions->getNumberOfJoints(limbId), movespeed);

    // compute the time required to move into the initial pose for each limb
    double moveTime = 1000*(maxDifference(currentPosition, targetPosition)/movespeed);

    // set the move complettion to be the maximum of each limb
    //movecompletiontime = m_data->CurrentTime + moveTime;

    // give the command to the actionators
    m_actions->addJointPositions(limbId, m_data->CurrentTime + 100, currentPosition, velocity, gain);
    m_actions->addJointPositions(limbId, m_data->CurrentTime + moveTime, targetPosition, velocity, gain);
}
