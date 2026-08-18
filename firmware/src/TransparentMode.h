#pragma once

#include <Arduino.h>


constexpr TransparentControllerParameters
    DEFAULT_TRANSPARENT_CONTROLLER_PARAMETERS
{
    .enabled 		= true,
    .input_hp_cutoff_hz       = 0.5, // Hz
    .input_lp_cutoff_hz      = 3.67, // Hz
    .derivative_lp_cutoff_hz = 5.0f,
    .friction_lp_cutoff_hz = 5.0f,
    .kp              = 0.1f,
    .kd              = 0.01f,
    .comp_torque = 0.8,
    .trigger_on_trq = 0.4,
    .trigger_off_trq = 0.7,
    .max_abs_out_trq = 0.3
};


#if defined(PLATFORM_TEENSY)

// =========================== FIRST ORDER HIGH PASS FILTER =====================================

class FirstOrderHighPass
{
public:
    FirstOrderHighPass(
        float cutoffHz,
        float sampleRateHz)
        :
        m_sampleRateHz(sampleRateHz)
    {
        setCutoff(cutoffHz);
    }

    void setCutoff(float cutoffHz)
    {
        if (cutoffHz <= 0.0f) {
            return;
        }

        m_cutoffHz = cutoffHz;

        const float dt =
            1.0f / m_sampleRateHz;

        const float tau =
            1.0f /
            (
                2.0f *
                PI *
                m_cutoffHz
            );

        m_alpha =
            tau /
            (
                tau + dt
            );
    }

    float update(float input)
    {
        if (!m_initialized)
        {
            m_previousInput = input;
            m_previousOutput = 0.0f;
            m_initialized = true;

            return 0.0f;
        }

        const float output =
            m_alpha *
            (
                m_previousOutput
                + input
                - m_previousInput
            );

        m_previousInput = input;
        m_previousOutput = output;

        return output;
    }

    void reset(float input = 0.0f)
    {
        m_previousInput = input;
        m_previousOutput = 0.0f;
        m_initialized = true;
    }

private:
    float m_sampleRateHz;
    float m_cutoffHz{0.0f};
    float m_alpha{0.0f};

    float m_previousInput{0.0f};
    float m_previousOutput{0.0f};

    bool m_initialized{false};
};

// =========================== FIRST ORDER LOW PASS FILTER ==========================================

class FirstOrderLowPass
{
public:
    FirstOrderLowPass(
        float cutoffHz,
        float sampleRateHz)
        :
        m_sampleRateHz(sampleRateHz)
    {
        setCutoff(cutoffHz);
    }

    void setCutoff(float cutoffHz)
    {
        if (cutoffHz <= 0.0f) {
            return;
        }

        m_cutoffHz = cutoffHz;

        m_alpha =
            1.0f -
            expf(
                -2.0f *
                PI *
                m_cutoffHz /
                m_sampleRateHz
            );
    }

    float update(float input)
    {
        if (!m_initialized)
        {
            m_value = input;
            m_initialized = true;
            return m_value;
        }

        m_value +=
            m_alpha *
            (
                input - m_value
            );

        return m_value;
    }

    void reset(float value = 0.0f)
    {
        m_value = value;
        m_initialized = true;
    }

private:
    float m_sampleRateHz;
    float m_cutoffHz{0.0f};
    float m_alpha{0.0f};

    float m_value{0.0f};
    bool m_initialized{false};
};

// =========================== MAIN TRANSPARENT MODE CONTROLLER =====================================

template<Side side>
class TransparentModeController final : public ITask
{
public:
    TransparentModeController(
        Topic<LoadCellTorques>& loadCells,
        Topic<MotorFeedback>& motorFeed,
        Topic<float>& motorCmd,
        Topic<TransparentControllerParameters>& params,
		Topic<float>& loadCellsIntermediateTorque
		)
        :
        m_loadcells(loadCells),
        m_motorFeed(motorFeed),
        m_motorCmd(motorCmd),
        m_params(params),
		m_loadCellsIntermediateTorque(loadCellsIntermediateTorque),
        m_inputHighPass(DEFAULT_TRANSPARENT_CONTROLLER_PARAMETERS.input_hp_cutoff_hz,1000.0f),
        m_inputLowPass(DEFAULT_TRANSPARENT_CONTROLLER_PARAMETERS.input_lp_cutoff_hz,1000.0f),
        m_derivativeLowPass(DEFAULT_TRANSPARENT_CONTROLLER_PARAMETERS.derivative_lp_cutoff_hz,1000.0f),
        m_frictionLowPass(DEFAULT_TRANSPARENT_CONTROLLER_PARAMETERS.friction_lp_cutoff_hz,1000.0f)
    {}


    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t PERIOD_US = 1'000; // 1 kHz

        if (nowUs - m_previousUpdateUs < PERIOD_US)
        {
            return;
        }

        m_previousUpdateUs = nowUs;

        if (!m_loadcells.valid() || !m_params.valid())
        {
            return;
        }
        const auto& params =
            m_params.latest();

        if (m_params.sequence()!= m_lastParamsSequence)
        {
            m_lastParamsSequence =
                m_params.sequence();


            m_inputHighPass.setCutoff(
                params.input_hp_cutoff_hz
            );

            m_inputLowPass.setCutoff(
                params.input_lp_cutoff_hz
            );

            m_derivativeLowPass.setCutoff(
                params.derivative_lp_cutoff_hz
            );

            m_frictionLowPass.setCutoff(
                params.friction_lp_cutoff_hz
            );
        }


        if (!params.enabled)
        {
            return;
        }


        const auto& loadcellSnapshot = m_loadcells.latest();

        const float measuredTorque = loadcellSnapshot[static_cast<uint8_t>(side)]; // to differentiate left and right


        // =====================================================
        // INITIALIZATION
        // =====================================================

        if (!m_initialized)
        {
            /*
             * Initialize the HP input history to the
             * current measured value.
             *
             * This prevents the initial DC offset from
             * appearing as a huge transient.
             */

            m_inputHighPass.reset(
                measuredTorque
            );

            m_inputLowPass.reset(
                0.0f
            );

            m_filteredTorque =
                0.0f;

            m_previousFilteredTorque =
                0.0f;

            m_filteredTorqueDerivative =
                0.0f;

            m_filteredFrictionComp =
                0.0f;

            m_previousUs =
                nowUs;

            m_initialized =
                true;

            return;
        }


        const float dt =
            static_cast<float>(
                nowUs - m_previousUs
            )
            *
            1e-6f;

        if (dt <= 0.0f)
        {
            return;
        }


        // =====================================================
        // 1) HIGH-PASS FILTER
        //
        // 1st-order (Butterworth??)_
        // fc = set by m_parans
        // fs = 1000 Hz
        // =====================================================

        const float highPassTorque = m_inputHighPass.update(measuredTorque); //note feed through the HPF here


        // =====================================================
        // 2) LOW-PASS FILTER
        //
        // 1st order (ButterWorth??)
        // fc = set by m_parans
        // fs = 1000 Hz
        // =====================================================

        const float lowPassTorque = m_inputLowPass.update(highPassTorque); //note feed through the LPF here

        m_loadCellsIntermediateTorque.publish(
        lowPassTorque
        );

        m_filteredTorque = lowPassTorque;


        // =====================================================
        // TORQUE DERIVATIVE
        // =====================================================

        const float rawTorqueDerivative =
            (
                m_filteredTorque
                -
                m_previousFilteredTorque
            )
            /
            dt;


        // =====================================================
        // DERIVATIVE LOW-PASS FILTER
        // =====================================================

        m_filteredTorqueDerivative =
        m_derivativeLowPass.update(
            rawTorqueDerivative
        );


        const float error =
            -m_filteredTorque;

        const float errorDerivative =
            -m_filteredTorqueDerivative;


        // =====================================================
        // PD CONTROLLER
        // =====================================================

        const float feedbackTorque =
            params.kp
            *
            error
            +
            params.kd
            *
            errorDerivative;


        // =====================================================
        // FRICTION COMPENSATION
        // =====================================================

        updateFrictionDirection(
            feedbackTorque,
            params
        );


        float targetFrictionComp =
            0.0f;

        if (
            m_frictionDirection > 0
        )
        {
            targetFrictionComp =
                params.comp_torque;
        }
        else if (
            m_frictionDirection < 0
        )
        {
            targetFrictionComp =
                -params.comp_torque;
        }


        m_filteredFrictionComp =
        m_frictionLowPass.update(
            targetFrictionComp
        );


        float commandedTorque =
            feedbackTorque
            +
            m_filteredFrictionComp;


        commandedTorque =
            constrain(
                commandedTorque,
                -params.max_abs_out_trq,
                params.max_abs_out_trq
            );


        m_motorCmd.publish(
            commandedTorque
        );


        // =====================================================
        // UPDATE STATE
        // =====================================================

        m_previousFilteredTorque =
            m_filteredTorque;

        m_previousUs =
            nowUs;
    }


private:


    void updateFrictionDirection(
        float feedbackTorque,
        const TransparentControllerParameters& params)
    {
        if (
            feedbackTorque >
            params.trigger_on_trq
        )
        {
            m_frictionDirection = +1;
            return;
        }

        if (
            feedbackTorque <
            -params.trigger_on_trq
        )
        {
            m_frictionDirection = -1;
            return;
        }

        if (
            std::abs(feedbackTorque) <
            params.trigger_off_trq
        )
        {
            m_frictionDirection = 0;
        }
    }


private:



    // =========================================================
    // TOPICS
    // =========================================================

    uint32_t m_lastParamsSequence{0};

    Topic<LoadCellTorques>&
        m_loadcells;

    Topic<MotorFeedback>&
        m_motorFeed;

    Topic<float>&
        m_motorCmd;

    Topic<TransparentControllerParameters>&
        m_params;

	Topic<float>&
    	m_loadCellsIntermediateTorque;


    // =========================================================
    // FILTER OBJECTS
    // =========================================================


    FirstOrderHighPass m_inputHighPass;
    FirstOrderLowPass  m_inputLowPass;
    FirstOrderLowPass  m_derivativeLowPass;
    FirstOrderLowPass  m_frictionLowPass;

    // =========================================================
    // LOW-PASS FILTER STATE
    // =========================================================


    // =========================================================
    // CONTROLLER STATE
    // =========================================================

    float
        m_filteredTorque{0.0f};

    float
        m_previousFilteredTorque{0.0f};

    float
        m_filteredTorqueDerivative{0.0f};

    float
        m_filteredFrictionComp{0.0f};

    int
        m_frictionDirection{0};

    uint32_t
        m_previousUs{0};

    uint32_t
        m_previousUpdateUs{0};

    bool
        m_initialized{false};
};

#else
#endif