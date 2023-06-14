/*
  AISFC Core Management header

  This header is the master handler for the FC's capabilitiy.
    **This is Mission Critical**
  Master functions that coordinate between resources and modules are handled here

  Created: 23th May 2023
  Last Update: 23th May 2023
  Created By: Michael Haggart
  For: StarthAIS
  Updated by: Michael Haggart
              Leon Yip
              Preben Rasmussen
              #Add New Names Here
*/

#ifndef AISFCCore
#define AISFCCore
//
//
enum flightStatus
{
    preLaunch,
    Boost,
    Coast,
    Apogee,
    Drogue,
    Main,
    Landed
};
//
//
bool descendingCheck(int& sampleCount, float highest_Alt, float cAlt);
bool motorCheckFunction(int& accelSampleCount, float absoluteAccel);
flightStatus stateCheckFunc(flightStatus& fS, float timeSinceLaunch, bool apogeeCheck, bool drogueDep, bool mainDep, bool motorCheck, float highestAlt, float curAlt);
//
//
bool descendingCheck(int& sampleCount, float highest_Alt, float cAlt) {
    if ((highest_Alt - cAlt) >= 1)  //if current alt is lower than 20 meter below apogee
    {
        sampleCount = sampleCount + 1;  //counter increments
        highest_Alt = cAlt;               //apogee = current alt
        if (sampleCount == 15)          //if 15 counts are sucessful return true;
        {
            return true;
        }
        return false;
    }
    if ((highest_Alt - cAlt) < 1)  //if current alt is greater than 1 meter above apogee
    {
        highest_Alt = highest_Alt;  //apogee
        sampleCount = 0;
        return false;
    }
}
//
//
bool motorCheckFunction(int& accelSampleCount, float absoluteAccel)
{
    bool motorCheck = false;
    if (absoluteAccel > 1.2)
    {
        accelSampleCount = accelSampleCount + 1;
        if (accelSampleCount == 5)
        {
            return motorCheck = true;
        }
        return motorCheck = false;
    }
    else
    {
        accelSampleCount = 0;
        return motorCheck = false;
    }
}
//
//
flightStatus stateCheckFunc(flightStatus& fS, float timeSinceLaunch, bool apogeeCheck, bool drogueDep, bool mainDep, bool motorCheck, float highestAlt, float curAlt)
{
    //The motorCheck is bool derived from the current acceleration of the rocket. If it is above some threshhold, then the motor is currently firing
    //If this is true, then the rocket is in its boost phase
    //If this is false && apogeeCheck is false, the rocket is in its Coast phase. 
    if (apogeeCheck == false && motorCheck == false && drogueDep == false && mainDep == false && highestAlt >= 0 && curAlt <= 50)
    {
        return fS = flightStatus::preLaunch; //0
    }
    if (fS == 0 && apogeeCheck == false && motorCheck == true && drogueDep == false && mainDep == false && highestAlt >= 0 && curAlt >= 0){
        return fS = flightStatus::Boost;  //1
    }
    if (fS == 1 && apogeeCheck == false && motorCheck == true && drogueDep == false && mainDep == false && highestAlt >= 0 && curAlt >= 4)
    {
        return fS = flightStatus::Coast;  //2
    }
    if (fS == 2 && apogeeCheck == true && motorCheck == true && drogueDep == false && mainDep == false && highestAlt >= 0 ) //removed curAlt >= 12
    {
        return fS = flightStatus::Apogee; //3
    }
    if (fS == 3 && apogeeCheck == true && motorCheck == true && drogueDep == true && mainDep == false && highestAlt >= curAlt) // && curAlt >= 10
    {
        return fS = flightStatus::Drogue; //4
    }
    if (fS == 4 && apogeeCheck == true && motorCheck == true && drogueDep == true && mainDep == false && highestAlt >= curAlt && (curAlt >= 1 && curAlt <=4))
    {
        return fS = flightStatus::Main; //5
    }
    if (fS == 5 && apogeeCheck == true && motorCheck == true && drogueDep == true && mainDep == true && highestAlt >= curAlt && curAlt <= 1)
    {
        return fS = flightStatus::Landed; //6
    }
    //if (apogeeCheck == true && motorCheck == true && drogueDep == false && mainDep == false && curAlt >= 350)
    //{
    //    //CREATE EMERGENCY FLIGHT STATUS TO DEPLOY BOTH DROGUE AND MAIN IF CRASHING 
    //}
    else
    {
        return fS = fS;
    }
  }


//
//
#endif // !AISFCCore