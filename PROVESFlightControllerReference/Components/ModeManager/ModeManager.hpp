// ======================================================================
// \title  ModeManager.hpp
// \author Auto-generated
// \brief  hpp file for ModeManager component implementation class
// ======================================================================

#ifndef Components_ModeManager_HPP
#define Components_ModeManager_HPP

#include <Os/File.hpp>

#include "Fw/Types/String.hpp"
#include "PROVESFlightControllerReference/Components/ModeManager/ModeManagerComponentAc.hpp"

namespace Components {

class ModeManager : public ModeManagerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct ModeManager object
    ModeManager(const char* const compName  //!< The component name
    );

    //! Destroy ModeManager object
    ~ModeManager();

    //! Initialize the component
    void init(FwSizeType queueDepth,        //!< Queue depth for async ports
              FwEnumStoreType instance = 0  //!< Instance ID
    );

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for completeSequence
    //!
    //! Port receiving completion status from the safe mode sequence
    void completeSequence_handler(FwIndexType portNum,             //!< The port number
                                  FwOpcodeType opCode,             //!< The opcode (unused)
                                  U32 cmdSeq,                      //!< The command sequence number (unused)
                                  const Fw::CmdResponse& response  //!< The command response
                                  ) override;

    //! Handler implementation for forceSafeMode
    //!
    //! Port to force safe mode entry (callable by other components)
    //! @param reason The reason for entering safe mode (NONE defaults to EXTERNAL_REQUEST)
    void forceSafeMode_handler(FwIndexType portNum,                      //!< The port number
                               const Components::SafeModeReason& reason  //!< The safe mode reason
                               ) override;
                               
    //! Handler implementation for forceNormalMode
    //!
    //! Port to force Normal Mode entry (callable by other components)
    void forceNormalMode_handler(FwIndexType portNum    //!< The port number
                               ) override;

    //! Handler implementation for getMode
    //!
    //! Port to query the current system mode
    Components::SystemMode getMode_handler(FwIndexType portNum  //!< The port number
                                           ) override;

    //! Handler implementation for prepareForReboot
    //!
    //! Port called before intentional reboot to set clean shutdown flag
    void prepareForReboot_handler(FwIndexType portNum  //!< The port number
                                  ) override;

    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command FORCE_SAFE_MODE
    void FORCE_SAFE_MODE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                    U32 cmdSeq            //!< The command sequence number
                                    ) override;

    //! Handler implementation for command EXIT_SAFE_MODE
    void EXIT_SAFE_MODE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                   U32 cmdSeq            //!< The command sequence number
                                   ) override;

    //! Handler implementation for command GET_CURRENT_MODE
    void GET_CURRENT_MODE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                     U32 cmdSeq            //!< The command sequence number
                                     ) override;

    //! Handler implementation for command GET_SAFE_MODE_REASON
    void GET_SAFE_MODE_REASON_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                         U32 cmdSeq            //!< The command sequence number
                                         ) override;

  private:
    // ----------------------------------------------------------------------
    // Private helper methods
    // ----------------------------------------------------------------------

    //! Load persistent state from file
    void loadState();

    //! Save persistent state to file
    void saveState();

    //! Enter safe mode with specified reason
    void enterSafeMode(Components::SafeModeReason reason);

    //! Exit safe mode (manual command)
    void exitSafeMode();

    //! Turn off non-critical components
    void turnOffNonCriticalComponents();

    //! Turn on components (restore normal operation)
    void turnOnComponents();

    // run the safe mode seauence
    void runSafeModeSequence();

    // ----------------------------------------------------------------------
    // Private enums and types
    // ----------------------------------------------------------------------

    //! System mode enumeration
    enum class SystemMode : U8 { SAFE_MODE = 1, NORMAL = 2 };

    //! Persistent state structure
    struct PersistentState {
        U8 mode;                 //!< Current mode (SystemMode)
        U32 safeModeEntryCount;  //!< Number of times safe mode entered
        U8 safeModeReason;       //!< Reason for safe mode entry (SafeModeReason)
        U8 cleanShutdown;        //!< Clean shutdown flag (1 = clean, 0 = unclean)
    };

    // ----------------------------------------------------------------------
    // Private member variables
    // ----------------------------------------------------------------------

    SystemMode m_mode;                            //!< Current system mode
    U32 m_safeModeEntryCount;                     //!< Counter for safe mode entries
    Components::SafeModeReason m_safeModeReason;  //!< Current safe mode reason

    // ----------------------------------------------------------------------
    // Constants
    // ----------------------------------------------------------------------

    static constexpr const char* STATE_FILE_PATH = "/mode_state.bin";  //!< State file path
};

}  // namespace Components

#endif
