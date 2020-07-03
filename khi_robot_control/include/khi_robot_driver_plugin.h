/*!
  \file		khi_robot_control_plugin.h
  \author	Toshio UESHIBA
*/
#ifndef KHI_ROBOT_CONTROL_PLUGIN_H
#define KHI_ROBOT_CONTROL_PLUGIN_H

#include <cstdint>
#include <array>
#include <khi_robot_driver.h>

namespace khi_robot_control
{
/************************************************************************
*  class DriverPlugin							*
************************************************************************/
class DriverPlugin
{
  public:
    using dio_t	= std::array<uint8_t, 64>;

  public:
    void		init(int cont_no, KhiRobotDriver* driver)	;

  protected:
    void		clear()						;
    bool		read()						;
    void		write()					const	;

  private:
    virtual void	onInit()					= 0;

  protected:
    dio_t		_in, _out, _mask;

  private:
    int			_cont_no;
    KhiRobotDriver*	_driver;
};

inline void
DriverPlugin::init(int cont_no, KhiRobotDriver* driver)
{
    _cont_no = cont_no;
    _driver  = driver;

    onInit();
}

inline void
DriverPlugin::clear()
{
    std::fill(std::begin(_out),  std::end(_out),  0);
    std::fill(std::begin(_mask), std::end(_mask), 0);
}

inline bool
DriverPlugin::read()
{
    return _driver->getDIO(_cont_no, _in.data(), _out.data());
}

inline void
DriverPlugin::write() const
{
    _driver->setDIO(_cont_no, _out.data(), _mask.data());
}

}	// namespace khi_robot_control
#endif	// !KHI_ROBOT_CONTROL_PLUGIN_H
