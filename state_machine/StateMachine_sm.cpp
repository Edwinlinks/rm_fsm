//
// ex: set ro:
// DO NOT EDIT.
// generated by smc (http://smc.sourceforge.net/)
// from file : StateMachine.sm
//

#include "rm_fsm/StateMachine.h"
#include "/home/dynamicx/rm_ws/src/rm_software/rm_fsm/state_machine/StateMachine_sm.h"

using namespace statemap;

// Static class declarations.
StateMachineMap_Idle StateMachineMap::Idle("StateMachineMap::Idle", 0);
StateMachineMap_Raw StateMachineMap::Raw("StateMachineMap::Raw", 1);
StateMachineMap_Calibrate StateMachineMap::Calibrate("StateMachineMap::Calibrate", 2);
StateMachineMap_Standby StateMachineMap::Standby("StateMachineMap::Standby", 3);
StateMachineMap_Cruise StateMachineMap::Cruise("StateMachineMap::Cruise", 4);

void StateMachineState::dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_)
{
    Default(context);
}

void StateMachineState::refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_)
{
    Default(context);
}

void StateMachineState::Default(StateMachineContext& context)
{
    throw (
        TransitionUndefinedException(
            (context.getState()).getName(),
            context.getTransition()));

}

void StateMachineMap_Idle::dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.getCalibrateStatus() == false )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initCalibrate();
            context.setState(StateMachineMap::Calibrate);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Calibrate);
            throw;
        }
        context.getState().Entry(context);
    }
    else if ( ctxt.isRaw(data_dbus_) == true )

    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initRaw();
            context.setState(StateMachineMap::Raw);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Raw);
            throw;
        }
        context.getState().Entry(context);
    }

}

void StateMachineMap_Idle::refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isStandby(referee_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initStandby();
            context.setState(StateMachineMap::Standby);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Standby);
            throw;
        }
        context.getState().Entry(context);
    }
    else if ( ctxt.isCruise(referee_) == true )

    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initCruise();
            context.setState(StateMachineMap::Cruise);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Cruise);
            throw;
        }
        context.getState().Entry(context);
    }    else
    {
         StateMachineMap_Default::refereeUpdate(context, referee_);
    }


}

void StateMachineMap_Raw::Entry(StateMachineContext& context)

{
    StateMachine& ctxt = context.getOwner();

    ctxt.rawChassis();
}

void StateMachineMap_Raw::dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isCalibrate(data_dbus_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initCalibrate();
            context.setState(StateMachineMap::Calibrate);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Calibrate);
            throw;
        }
        context.getState().Entry(context);
    }
    else
    {
        StateMachineState& endState = context.getState();

        context.clearState();
        try
        {
            ctxt.sendRawCommand(ros::Time::now());
            context.setState(endState);
        }
        catch (...)
        {
            context.setState(endState);
            throw;
        }
    }

}

void StateMachineMap_Raw::refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isStandby(referee_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initStandby();
            context.setState(StateMachineMap::Standby);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Standby);
            throw;
        }
        context.getState().Entry(context);
    }
    else if ( ctxt.isCruise(referee_) == true )

    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initCruise();
            context.setState(StateMachineMap::Cruise);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Cruise);
            throw;
        }
        context.getState().Entry(context);
    }    else
    {
         StateMachineMap_Default::refereeUpdate(context, referee_);
    }


}

void StateMachineMap_Calibrate::Entry(StateMachineContext& context)

{
    StateMachine& ctxt = context.getOwner();

    ctxt.calibrateChassis();
}

void StateMachineMap_Calibrate::dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isRaw(data_dbus_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initRaw();
            context.setState(StateMachineMap::Raw);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Raw);
            throw;
        }
        context.getState().Entry(context);
    }
    else
    {
        StateMachineState& endState = context.getState();

        context.clearState();
        try
        {
            ctxt.sendCalibrateCommand(ros::Time::now());
            context.setState(endState);
        }
        catch (...)
        {
            context.setState(endState);
            throw;
        }
    }

}

void StateMachineMap_Calibrate::refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isStandby(referee_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initStandby();
            context.setState(StateMachineMap::Standby);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Standby);
            throw;
        }
        context.getState().Entry(context);
    }
    else if ( ctxt.isCruise(referee_) == true )

    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initCruise();
            context.setState(StateMachineMap::Cruise);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Cruise);
            throw;
        }
        context.getState().Entry(context);
    }    else
    {
         StateMachineMap_Default::refereeUpdate(context, referee_);
    }


}

void StateMachineMap_Standby::Entry(StateMachineContext& context)

{
    StateMachine& ctxt = context.getOwner();

    ctxt.standbyChassis();
}

void StateMachineMap_Standby::dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isRaw(data_dbus_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initRaw();
            context.setState(StateMachineMap::Raw);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Raw);
            throw;
        }
        context.getState().Entry(context);
    }
    else
    {
        StateMachineState& endState = context.getState();

        context.clearState();
        try
        {
            ctxt.sendStandbyCommand(ros::Time::now());
            context.setState(endState);
        }
        catch (...)
        {
            context.setState(endState);
            throw;
        }
    }

}

void StateMachineMap_Standby::refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isCruise(referee_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initCruise();
            context.setState(StateMachineMap::Cruise);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Cruise);
            throw;
        }
        context.getState().Entry(context);
    }
    else
    {
        StateMachineState& endState = context.getState();

        context.clearState();
        try
        {
            ctxt.sendStandbyCommand(ros::Time::now());
            context.setState(endState);
        }
        catch (...)
        {
            context.setState(endState);
            throw;
        }
    }

}

void StateMachineMap_Cruise::Entry(StateMachineContext& context)

{
    StateMachine& ctxt = context.getOwner();

    ctxt.cruiseChassis();
}

void StateMachineMap_Cruise::dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isRaw(data_dbus_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initRaw();
            context.setState(StateMachineMap::Raw);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Raw);
            throw;
        }
        context.getState().Entry(context);
    }
    else
    {
        StateMachineState& endState = context.getState();

        context.clearState();
        try
        {
            ctxt.sendCruiseCommand(ros::Time::now());
            context.setState(endState);
        }
        catch (...)
        {
            context.setState(endState);
            throw;
        }
    }

}

void StateMachineMap_Cruise::refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_)
{
    StateMachine& ctxt = context.getOwner();

    if ( ctxt.isStandby(referee_) == true )
    {
        context.getState().Exit(context);
        context.clearState();
        try
        {
            ctxt.initStandby();
            context.setState(StateMachineMap::Standby);
        }
        catch (...)
        {
            context.setState(StateMachineMap::Standby);
            throw;
        }
        context.getState().Entry(context);
    }
    else
    {
        StateMachineState& endState = context.getState();

        context.clearState();
        try
        {
            ctxt.sendCruiseCommand(ros::Time::now());
            context.setState(endState);
        }
        catch (...)
        {
            context.setState(endState);
            throw;
        }
    }

}

//
// Local variables:
//  buffer-read-only: t
// End:
//
