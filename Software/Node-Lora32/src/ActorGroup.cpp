#include "ActorGroup.h"

void ActorGroup::addActiveActor(Actor &activeActor)
{
    activeGroup.push_back(activeActor);
}

void ActorGroup::addPassiveActor(Actor &passiveActor)
{
    passiveGroup.push_back(passiveActor);
}

bool ActorGroup::checkValid(std::vector<Actor> &actorGroup)
{
    return !actorGroup.size() ? (Serial.println("There is no actor in this group. Pls add it"), false) : true;
}

void ActorGroup::turnOnActiveActors(uint delayTime)
{
    if (!checkValid(activeGroup))
        return;
    for (auto actor = activeGroup.begin(); actor != activeGroup.end(); ++actor)
    {
        actor->turnOn();
        delay(delayTime);
    }
}

void ActorGroup::turnOffActiveActors(uint delayTime)
{
    if (!checkValid(activeGroup))
        return;
    for (auto actor = activeGroup.begin(); actor != activeGroup.end(); ++actor)
    {
        actor->turnOff();
        delay(delayTime);
    }
}

void ActorGroup::turnOnPassiveActors(uint delayTime)
{
    if (!checkValid(passiveGroup))
        return;
    for (auto actor = passiveGroup.begin(); actor != passiveGroup.end(); ++actor)
    {
        actor->turnOn();
        delay(delayTime);
    }
}

void ActorGroup::turnOffPassiveActors(uint delayTime)
{
    if (!checkValid(passiveGroup))
        return;
    for (auto actor = passiveGroup.begin(); actor != passiveGroup.end(); ++actor)
    {
        actor->turnOff();
        delay(delayTime);
    }
}

void ActorGroup::turnOnAllActors(uint delayTime){
    turnOnPassiveActors();
    delay(delayTime);
    turnOnActiveActors();
}

void ActorGroup::turnOffAllActors(uint delayTime){
    turnOffActiveActors();
    delay(delayTime);
    turnOffPassiveActors();
}