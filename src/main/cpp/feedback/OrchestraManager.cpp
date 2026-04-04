#include "feedback/OrchestraManager.h"

OrchestraManager *OrchestraManager::m_instance = nullptr;

OrchestraManager *OrchestraManager::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new OrchestraManager();
    }
    return m_instance;
}

void OrchestraManager::Register(IOrchestraPlayable *playable)
{
    if (playable != nullptr)
    {
        m_playables.push_back(playable);
    }
}

void OrchestraManager::LoadMusic(const std::string &filePath)
{
    for (auto playable : m_playables)
    {
        playable->LoadMusic(filePath);
    }
}

void OrchestraManager::StartMusic()
{
    for (auto playable : m_playables)
    {
        playable->StartMusic();
    }
}

void OrchestraManager::StopMusic()
{
    for (auto playable : m_playables)
    {
        playable->StopMusic();
    }
}
