// ======================================================================
// \title  Old_ADCS.hpp
// \brief  hpp file for Old_ADCS component implementation class
// ======================================================================

#ifndef Components_Old_ADCS_HPP
#define Components_Old_ADCS_HPP

#include "PROVESFlightControllerReference/Components/Old_ADCS/Old_ADCSComponentAc.hpp"

namespace Components {

class Old_ADCS final : public Old_ADCSComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct Old_ADCS object
    Old_ADCS(const char* const compName  //!< The component name
    );

    //! Destroy Old_ADCS object
    ~Old_ADCS();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    //!
    //! Scheduled port for periodic temperature reading
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;
};

}  // namespace Components

#endif
