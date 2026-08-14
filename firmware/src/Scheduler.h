template<size_t MaxTasks>
class Scheduler
{
public:
    bool add(ITask& task)
    {
        if (m_count >= MaxTasks) {
            return false;
        }

        m_tasks[m_count++] = &task;
        return true;
    }

    void run(uint32_t nowUs)
    {
        for (size_t i = 0;
             i < m_count;
             ++i)
        {
            m_tasks[i]->update(nowUs);
        }
    }

private:
    ITask* m_tasks[MaxTasks]{};
    size_t m_count{0};
};
