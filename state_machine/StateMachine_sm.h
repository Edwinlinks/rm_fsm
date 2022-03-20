//
// ex: set ro:
// DO NOT EDIT.
// generated by smc (http://smc.sourceforge.net/)
// from file : StateMachine.sm
//

#ifndef STATEMACHINE_SM_H
#define STATEMACHINE_SM_H


#define SMC_USES_IOSTREAMS

#include <statemap.h>
#include <rm_common/referee/referee.h>
#include <rm_msgs/DbusData.h>

// Forward declarations.
class StateMachineMap;
class StateMachineMap_Idle;
class StateMachineMap_Raw;
class StateMachineMap_Calibrate;
class StateMachineMap_Standby;
class StateMachineMap_Cruise;
class StateMachineMap_Default;
class StateMachineState;
class StateMachineContext;
class StateMachine;

class StateMachineState :
    public statemap::State
{
public:

    StateMachineState(const char * const name, const int stateId)
    : statemap::State(name, stateId)
    {};

    virtual void Entry(StateMachineContext&) {};
    virtual void Exit(StateMachineContext&) {};

    virtual void dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_);
    virtual void refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_);

protected:

    virtual void Default(StateMachineContext& context);
};

class StateMachineMap
{
public:

    static StateMachineMap_Idle Idle;
    static StateMachineMap_Raw Raw;
    static StateMachineMap_Calibrate Calibrate;
    static StateMachineMap_Standby Standby;
    static StateMachineMap_Cruise Cruise;
};

class StateMachineMap_Default :
    public StateMachineState
{
public:

    StateMachineMap_Default(const char * const name, const int stateId)
    : StateMachineState(name, stateId)
    {};

};

class StateMachineMap_Idle :
    public StateMachineMap_Default
{
public:
    StateMachineMap_Idle(const char * const name, const int stateId)
    : StateMachineMap_Default(name, stateId)
    {};

    virtual void dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_);
    virtual void refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_);
};

class StateMachineMap_Raw :
    public StateMachineMap_Default
{
public:
    StateMachineMap_Raw(const char * const name, const int stateId)
    : StateMachineMap_Default(name, stateId)
    {};

    virtual void Entry(StateMachineContext&);
    virtual void dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_);
    virtual void refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_);
};

class StateMachineMap_Calibrate :
    public StateMachineMap_Default
{
public:
    StateMachineMap_Calibrate(const char * const name, const int stateId)
    : StateMachineMap_Default(name, stateId)
    {};

    virtual void Entry(StateMachineContext&);
    virtual void dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_);
    virtual void refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_);
};

class StateMachineMap_Standby :
    public StateMachineMap_Default
{
public:
    StateMachineMap_Standby(const char * const name, const int stateId)
    : StateMachineMap_Default(name, stateId)
    {};

    virtual void Entry(StateMachineContext&);
    virtual void dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_);
    virtual void refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_);
};

class StateMachineMap_Cruise :
    public StateMachineMap_Default
{
public:
    StateMachineMap_Cruise(const char * const name, const int stateId)
    : StateMachineMap_Default(name, stateId)
    {};

    virtual void Entry(StateMachineContext&);
    virtual void dbusUpdate(StateMachineContext& context, rm_msgs::DbusData data_dbus_);
    virtual void refereeUpdate(StateMachineContext& context, rm_msgs::Referee referee_);
};

class StateMachineContext :
    public statemap::FSMContext
{
public:

    explicit StateMachineContext(StateMachine& owner)
    : FSMContext(StateMachineMap::Idle),
      _owner(owner)
    {};

    StateMachineContext(StateMachine& owner, const statemap::State& state)
    : FSMContext(state),
      _owner(owner)
    {};

    virtual void enterStartState()
    {
        getState().Entry(*this);
        return;
    }

    inline StateMachine& getOwner()
    {
        return (_owner);
    };

    inline StateMachineState& getState()
    {
        if (_state == NULL)
        {
            throw statemap::StateUndefinedException();
        }

        return dynamic_cast<StateMachineState&>(*_state);
    };

    inline void dbusUpdate(rm_msgs::DbusData data_dbus_)
    {
        getState().dbusUpdate(*this, data_dbus_);
    };

    inline void refereeUpdate(rm_msgs::Referee referee_)
    {
        getState().refereeUpdate(*this, referee_);
    };

private:
    StateMachine& _owner;
};


#endif // STATEMACHINE_SM_H

//
// Local variables:
//  buffer-read-only: t
// End:
//
