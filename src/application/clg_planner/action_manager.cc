/*
 * action_manager.cc
 *
 *  Created on: 20-12-2013
 *      Author: spiatek
 */

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>

#include "action_manager.h"

action_manager::action_manager() {
}

action_manager::~action_manager() {
}

bool action_manager::observe_color(std::vector<std::string> &p/*std::string col, std::string obj*/)
{
	boost::mutex::scoped_lock lock(mp_mutex);
	std::cout << "OBSERVE_COLOR PROCESS" << std::endl;
	//sprawdzam czy obiekt obj ma kolor col
	//przeszukuję mapę obiektów w poszukiwaniu obiektu o danym kolorze
	//później już zawsze traktuję ten obiekt jako obj
	return true;
}

bool action_manager::observ(std::vector<std::string> &p/*std::string obj, std::string pos*/)
{
	boost::mutex::scoped_lock lock(mp_mutex);
	std::cout << "OBSERV PROCESS" << std::endl;
	//szukam obiektu w obrazie
	//jak tylko znajdę obiekt, zapisuję jego pozycję w mapie obiektów
	//przy okazji zapisuję też jego kolor
	return true;
}

bool action_manager::move(std::vector<std::string> &p/*std::string pos1, std::string pos2*/)
{
	boost::mutex::scoped_lock lock(mp_mutex);
	std::cout << "MOVE PROCESS" << std::endl;
	//przemieszczam ramię manipulatora z pozycji 1 na 2
	return true;
}

bool action_manager::pickup(std::vector<std::string> &p/*std::string obj, std::string pos*/)
{
	boost::mutex::scoped_lock lock(mp_mutex);
	std::cout << "PICKUP PROCESS" << std::endl;
	//podnoszę obiekt obj umiejscowiony na konkretnej pozycji
	//1. jadę nad obj
	//2. na sztywno idę w dół i nabijam
	//3. podnoszę do góry
	return true;
}

bool action_manager::putdown(std::vector<std::string> &p/*std::string obj, std::string pos, std::string col*/)
{
	boost::mutex::scoped_lock lock(mp_mutex);
	std::cout << "PUTDOWN PROCESS" << std::endl;
	//stawiam obiekt obj o kolorze col w miejscu pos
	return true;
}
