#ifndef     TRIGGER_COUNTER_H
#define     TRIGGER_COUNTER_H

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class   TriggerCounter
{
public:

    TriggerCounter() = default;

    ~TriggerCounter() = default;

    /// Устанавливать данный тригер исключительно в ситуации,
    /// когда вход меняется с ложного значения на истинное
    void set(bool input)
    {
        if (!prev_input && input)
        {
            state = true;
        }

        prev_input = input;
    }

    /// Сброс
    void reset()
    {
        state = false;
    }

    /// Вернуть текущее состояние
    bool getState() const
    {
        return state;
    }

protected:

    bool prev_input = true;

    bool state = false;
};


#endif
