#pragma once
#include <vector>
#include "Actor.h"

class ActorGroup
{
private:
    //At the moment we have 2 kind of Actors: Pump and Ventil.
    //Pump is belong to Active Actor Group: trigger System.
    //Ventil is belong to Passive Actor Group: re-direct System.
    //For easy extent this system with multiple Pumps and Ventils
    //we create a list to control all of them with same principle
    //Active Group: e.g Pumps
    std::vector<Actor> activeGroup;
    /*
    At the moment we have 2 kind of Actors: Pump and Ventil.
    Pump is belong to Active Actor Group: trigger System.
    Ventil is belong to Passive Actor Group: re-direct System.
    For easy extent this system with multiple Pumps and Ventils
    we create a list to control all of them with same principle
    Passive Group: e.g Ventils
    */
    std::vector<Actor> passiveGroup;

public:
    ActorGroup(){};
    ~ActorGroup(){};

    bool checkValid(std::vector<Actor> &actorGroup);

    void addActiveActor(Actor &activeActor);
    void addPassiveActor(Actor &passiveActor);

    void turnOnActiveActors(uint delayTime = 100);
    void turnOffActiveActors(uint delayTime = 100);

    void turnOnPassiveActors(uint delayTime = 100);
    void turnOffPassiveActors(uint delayTime = 100);

    ///To turn on/off 2 kind of actor group at ones, we should add delay time on it,
    ///to avoid dynamic pressure creating in pipe, which destroy system easily
    ///For safety, turn on all passive actors first, then delay for a while
    ///then turn on all active sensors
    void turnOnAllActors(uint delayTime = 2000);
    /**
    *To turn on/off 2 kind of actor group at ones, we should add delay time on it,
    *to avoid dynamic pressure creating in pipe, which destroy system easily
    *For safety, turn off all active actors first, then delay for a while
    *then turn off all passive sensors
    */
    void turnOffAllActors(uint delayTime = 2000);

    
};