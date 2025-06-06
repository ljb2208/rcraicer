/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file Diagnostics.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief Diagnostics class implementation
 *
 ***********************************************/

#include "../include/rcraicer/diagnostics.h"

#include <diagnostic_updater/publisher.hpp>
#include <stdio.h>
#include <sstream>

Diagnostics::Diagnostics()
{
    
}

Diagnostics::Diagnostics(rclcpp::Node::SharedPtr node,
                         const std::string otherInfo,
                         const std::string hardwareID,
                         const std::string hardwareLocation) :                         
  m_hardwareLocation(hardwareLocation),
  m_overallLevel(diagnostic_msgs::msg::DiagnosticStatus::OK)
{
    init(node, otherInfo, hardwareID, hardwareLocation);
}

Diagnostics::~Diagnostics()
{
  delete m_updater;
}


void Diagnostics::init(rclcpp::Node::SharedPtr node,
                       const std::string& otherInfo,
                       const std::string& hardwareID,
                       const std::string& hardwareLocation)
{  
  m_nodeHandle = node;
  m_hardwareLocation = hardwareLocation;
  m_overallLevel = diagnostic_msgs::msg::DiagnosticStatus::OK;

  m_nodeHandle->declare_parameter("diagnosticsFrequency", 1);
  int diagFreq = m_nodeHandle->get_parameter("diagnosticsFrequency").as_int();  

  m_updater = new diagnostic_updater::Updater(m_nodeHandle, diagFreq);
  m_updater->setHardwareID(hardwareID);
  m_updater->add(otherInfo, this, &Diagnostics::diagnostics);

  // m_heartbeatTimer = m_nodeHandle->create_wall_timer(std::chrono::milliseconds(1000/diagFreq), std::bind(&Diagnostics::diagUpdate, this));
  m_statusTimer = m_nodeHandle->create_wall_timer(std::chrono::milliseconds(1000/diagFreq), std::bind(&Diagnostics::diagnosticStatus, this));

}

void Diagnostics::diag(const std::string key, const std::string value, bool lock)
{
  if (lock) m_dataMutex.lock();
  m_diags[key] = value;
  if (lock) m_dataMutex.unlock();
}

void Diagnostics::diag_ok(const std::string msg)
{
  m_dataMutex.lock();
  m_diagMsgs[msg] = diagnostic_msgs::msg::DiagnosticStatus::OK;
  m_dataMutex.unlock();
}

void Diagnostics::diag_warn(const std::string msg)
{
  m_dataMutex.lock();
  m_diagMsgs[msg] = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  m_dataMutex.unlock();
}

void Diagnostics::diag_error(const std::string msg)
{
  m_dataMutex.lock();
  m_diagMsgs[msg] = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  m_dataMutex.unlock();
}

void Diagnostics::diagUpdate()
{
  //force the publishing of a diagnostics array based on the desired frequency
  m_updater->force_update();
}

void Diagnostics::OK()
{
  m_dataMutex.lock();
  m_overallLevel = diagnostic_msgs::msg::DiagnosticStatus::OK;
  m_dataMutex.unlock();
}

void Diagnostics::WARN()
{
    m_dataMutex.lock();
  m_overallLevel = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  m_dataMutex.unlock();
}

void Diagnostics::ERROR()
{
  m_dataMutex.lock();
  m_overallLevel = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  m_dataMutex.unlock();
}

void Diagnostics::tick(const std::string &name)
{
  std::map<std::string, std::vector<std::pair<int, rclcpp::Time> > >::iterator mapIt;
  m_dataMutex.lock();
  if( (mapIt = m_ticks.find(name)) == m_ticks.end())
  {
    std::vector<std::pair<int, rclcpp::Time> > toAdd;
    toAdd.push_back(std::pair<int, rclcpp::Time>(0, m_nodeHandle->now()) );

    mapIt = m_ticks.insert(std::pair<std::string,
                           std::vector<std::pair<int, rclcpp::Time> > >
                           (name, toAdd)).first;
  }
  ++mapIt->second.back().first;
  m_dataMutex.unlock();
}

void Diagnostics::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  //add current overall diagnostic level and message
  stat.summary(m_overallLevel, m_hardwareLocation);

  //Frequency messages are added to diagnotics only if tick() is being called
  std::map<std::string, std::vector<std::pair<int, rclcpp::Time> > >::iterator mapItF;
  rclcpp::Time n = m_nodeHandle->now();

  m_dataMutex.lock();
  for( mapItF = m_ticks.begin(); mapItF != m_ticks.end(); ++mapItF)
  {
    //remove all tick counts older than 15 seconds
    while( (n-mapItF->second.front().second).seconds() > 20.0)
    {
      mapItF->second.erase(mapItF->second.begin());
    }

    //sum all ticks in the window
    std::vector<std::pair<int, rclcpp::Time> >::const_iterator qIt;
    int sum = 0;
    for( qIt = mapItF->second.begin(); qIt != mapItF->second.end(); ++qIt)
    {
      sum += qIt->first;
    }
    
    //add a diagnostic message with the publishing freq over the sliding window
    double val = sum/(n-mapItF->second.front().second).seconds();
    if(!std::isnan(val) && !std::isinf(val))
    { //strs << sum/((n-mapItF->second.front().second).toSec());
      diag(mapItF->first + " freq(hz):", std::to_string(val), false);
    }
    //add new tick entry to vector
    mapItF->second.push_back( std::pair<int, rclcpp::Time>(0, n));
  }

  //add all queued diganostic messages, clear the queues
  std::map<std::string, char>::iterator mapIt;
  for(mapIt = m_diagMsgs.begin(); mapIt != m_diagMsgs.end(); ++mapIt)
  {
    stat.add(mapIt->first, mapIt->second);
  }
  m_diagMsgs.clear();

  std::map<std::string, std::string>::iterator mapIt2;
  for(mapIt2 = m_diags.begin(); mapIt2 != m_diags.end(); ++mapIt2)
  {
    stat.add(mapIt2->first, mapIt2->second);
  }
  m_diags.clear();
  m_dataMutex.unlock();
}
