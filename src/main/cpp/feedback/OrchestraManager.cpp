//====================================================================================================================================================
// Copyright 2026 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include "feedback/OrchestraManager.h"
#include <algorithm>
#include <filesystem>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <map>
#include <vector>

OrchestraManager *OrchestraManager::m_instance = nullptr;

OrchestraManager *OrchestraManager::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new OrchestraManager();
    }
    return m_instance;
}

template <typename T>
void OrchestraManager::AddInstrument(T *instrument)
{
    if (instrument != nullptr)
    {
        // Store the instrument adders so we can build orchestras later in LoadMusic
        m_instrumentAdders.emplace_back([instrument](ctre::phoenix6::Orchestra *orch)
                                        {
            if (orch != nullptr) {
                orch->AddInstrument(*instrument);
            } });
    }
}

template void OrchestraManager::AddInstrument<ctre::phoenix6::hardware::TalonFX>(ctre::phoenix6::hardware::TalonFX *instrument);
template void OrchestraManager::AddInstrument<ctre::phoenix6::hardware::TalonFXS>(ctre::phoenix6::hardware::TalonFXS *instrument);

void OrchestraManager::LoadMusic(const std::string &filePath)
{
    m_currentFilePath = filePath;
    m_musicLoaded = std::filesystem::exists(m_currentFilePath);

    // Clean up old orchestras
    for (auto orch : m_orchestras)
    {
        delete orch;
    }
    m_orchestras.clear();

    if (!m_musicLoaded)
        return;

    // Determine track count from map
    int tracks = 1; // Default to 1 track per orchestra if not found
    OrchestraMap orchestraMap;
    auto it = orchestraMap.m_map.find(filePath);
    if (it != orchestraMap.m_map.end())
    {
        tracks = it->second;
    }

    // Chunk instruments into multiple orchestras based on track count
    ctre::phoenix6::Orchestra *currentOrchestra = nullptr;
    int instrumentsInCurrentOrch = 0;

    for (auto &adder : m_instrumentAdders)
    {
        if (currentOrchestra == nullptr || instrumentsInCurrentOrch >= tracks)
        {
            currentOrchestra = new ctre::phoenix6::Orchestra();
            m_orchestras.push_back(currentOrchestra);
            instrumentsInCurrentOrch = 0;
        }
        adder(currentOrchestra);
        instrumentsInCurrentOrch++;
    }

    for (auto orch : m_orchestras)
    {
        orch->LoadMusic(m_currentFilePath.c_str());
    }
}

void OrchestraManager::StartMusic()
{
    if (m_musicLoaded && !m_orchestras.empty())
    {
        for (auto orch : m_orchestras)
        {
            orch->Play();
        }
    }
    else if (m_musicLoaded && m_orchestras.empty() && !m_currentFilePath.empty())
    {
        LoadMusic(m_currentFilePath);
        for (auto orch : m_orchestras)
        {
            orch->Play();
        }
    }
}

void OrchestraManager::StopMusic()
{
    for (auto orch : m_orchestras)
    {
        if (orch != nullptr)
        {
            orch->Stop();
            delete orch;
        }
    }
    m_orchestras.clear();
}
