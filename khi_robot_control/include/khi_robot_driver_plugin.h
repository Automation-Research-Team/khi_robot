/*!
  \file		khi_robot_control_plugin.h
  \author	Toshio UESHIBA
*/
#ifndef KHI_ROBOT_DRIVER_PLUGIN_H
#define KHI_ROBOT_DRIVER_PLUGIN_H

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
    using dio_t	= std::array<uint8_t, 64>;	// 64*8 = 512 signals

  public:
		DriverPlugin()	:_cont_no(0), _driver(nullptr)		{}

    void	init(int cont_no, KhiRobotDriver* driver)		;

  protected:
    void	clear()							;
    bool	read()							;
    bool	write()						 const	;
    bool	pulse(double sec=0.2)					;
    template <class T>
    T		in(int offset)					 const	;
    template <class T>
    T		out(int offset)					 const	;
    template <class T>
    void	set_out(int offset, T val, T mask=~0)			;
    template <class T>
    void	clear_mask(int offset)					;
    bool	set_signal(int sig, bool enable)		 const	;
    bool	set_bits(int sig, int nsigs, int val)		 const	;
    bool	pulse(int sig, double sec=0.2)			 const	;
    bool	set_variable(const std::string& name, int value) const	;
    bool	exec_as(const std::string& as_cmd)		 const	;
    bool	set_state_trigger(KhiRobotStateTrigger trigger)	 const	;
    int		get_state()					 const	;
    
  private:
    virtual void	onInit()					= 0;

  private:
    int			_cont_no;
    KhiRobotDriver*	_driver;
    dio_t		_in, _out, _mask;
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

inline bool
DriverPlugin::write() const
{
    return _driver->setDO(_cont_no, _out.data(), _mask.data());
}

inline bool
DriverPlugin::pulse(double sec)
{
    if (!write())
	return false;
    
    ros::Duration(sec).sleep();
    std::fill(std::begin(_out), std::end(_out), 0);

    return write();
}

template <class T> inline T
DriverPlugin::in(int offset) const
{
    return *reinterpret_cast<const T*>(_in.data() + offset);
}

template <class T> inline T
DriverPlugin::out(int offset) const
{
    return *reinterpret_cast<const T*>(_out.data() + offset);
}

template <class T> inline void
DriverPlugin::set_out(int offset, T val, T mask)
{
    *reinterpret_cast<T*>(_out.data()  + offset) = val;
    *reinterpret_cast<T*>(_mask.data() + offset) = mask;
}

template <class T> inline void
DriverPlugin::clear_mask(int offset)
{
    *reinterpret_cast<T*>(_mask.data() + offset) = T(0);
}

inline bool
DriverPlugin::set_signal(int sig, bool enable) const
{
    return set_bits(sig, 1, (enable ? 1 : 0));
}

inline bool
DriverPlugin::set_bits(int sig, int nsigs, int val) const
{
    return exec_as("BITS32 " + std::to_string(sig) + ',' +
		   std::to_string(nsigs) + '=' + std::to_string(val));
}

inline bool
DriverPlugin::pulse(int sig, double sec) const
{
    return exec_as("PULSE " + std::to_string(sig) + ',' + std::to_string(sec));
}

inline bool
DriverPlugin::set_variable(const std::string& name, int value) const
{
    return exec_as(name + '=' + std::to_string(value));

}

inline bool
DriverPlugin::exec_as(const std::string& as_cmd) const
{
    return _driver->exec_as(_cont_no, as_cmd);
}

inline bool
DriverPlugin::set_state_trigger(KhiRobotStateTrigger trigger) const
{
    return _driver->set_state_trigger(_cont_no, trigger);
}

inline int
DriverPlugin::get_state() const
{
    return _driver->getState(_cont_no);
}
    
}	// namespace khi_robot_control
#endif	// !KHI_ROBOT_DRIVER_PLUGIN_H
