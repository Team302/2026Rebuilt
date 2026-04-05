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
        m_instrumentAdders.emplace_back([instrument](ctre::phoenix6::Orchestra *orch)
                                        {
            if (orch != nullptr) {
                orch->AddInstrument(*instrument);
            } });

        if (m_orchestra != nullptr)
        {
            m_orchestra->AddInstrument(*instrument);
        }
    }
}

template void OrchestraManager::AddInstrument<ctre::phoenix6::hardware::TalonFX>(ctre::phoenix6::hardware::TalonFX *instrument);
template void OrchestraManager::AddInstrument<ctre::phoenix6::hardware::TalonFXS>(ctre::phoenix6::hardware::TalonFXS *instrument);

void OrchestraManager::LoadMusic(const std::string &filePath)
{
    m_currentFilePath = filePath;
    // m_musicLoaded = std::filesystem::exists(m_currentFilePath);
    m_musicLoaded = true;

    if (m_orchestra == nullptr)
    {
        m_orchestra = new ctre::phoenix6::Orchestra();
        for (auto &adder : m_instrumentAdders)
        {
            adder(m_orchestra);
        }
    }

    m_orchestra->LoadMusic(m_currentFilePath.c_str());
}

void OrchestraManager::StartMusic()
{
    if (m_musicLoaded && m_orchestra != nullptr)
    {
        m_orchestra->Play();
    }
    else if (m_musicLoaded && m_orchestra == nullptr && !m_currentFilePath.empty())
    {
        LoadMusic(m_currentFilePath);
        if (m_orchestra != nullptr)
        {
            m_orchestra->Play();
        }
    }
}

void OrchestraManager::StopMusic()
{
    if (m_orchestra != nullptr)
    {
        m_orchestra->Stop();
        delete m_orchestra;
        m_orchestra = nullptr;
    }
}
