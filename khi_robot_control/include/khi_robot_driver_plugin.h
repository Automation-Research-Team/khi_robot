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
    using dio_t	= std::array<uint8_t, 64>;	// 64*8 = 512 signals

  public:
		DriverPlugin()	:_cont_no(0), _driver(nullptr)		{}
    
    void	init(int cont_no, KhiRobotDriver* driver)		;

  protected:
    void	clear()							;
    bool	read()							;
    void	write()						const	;
    void	pulse(double sec=0.2)					;
    template <class T>
    T		in(int offfset)					const	;
    template <class T>
    T		out(int offfset)				const	;
    template <class T>
    void	set_out(int offfset, T val, T mask=~0)			;
    template <class T>
    void	clear_mask(int offfset)					;

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

inline void
DriverPlugin::write() const
{
    _driver->setDIO(_cont_no, _out.data(), _mask.data());
}

inline void
DriverPlugin::pulse(double sec)
{
    write();
    ros::Duration(sec).sleep();
    std::fill(std::begin(_out), std::end(_out), 0);
    write();
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
    
}	// namespace khi_robot_control
#endif	// !KHI_ROBOT_CONTROL_PLUGIN_H
