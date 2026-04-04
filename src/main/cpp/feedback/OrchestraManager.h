#pragma once
#include "feedback/IOrchestraPlayable.h"
#include <string>
#include <vector>

class OrchestraManager
{
public:
    static OrchestraManager *GetInstance();

    void Register(IOrchestraPlayable *playable);
    void LoadMusic(const std::string &filePath);
    void StartMusic();
    void StopMusic();

private:
    OrchestraManager() = default;
    ~OrchestraManager() = default;

    static OrchestraManager *m_instance;
    std::vector<IOrchestraPlayable *> m_playables;
};
