#include <boost/python.hpp>
#include "Kalman.h"

 
BOOST_PYTHON_MODULE(Kalman){
  using namespace boost::python;
  class_<Kalman>("Kalman", init<>())
    .def("getAngle", &Kalman::getAngle)
    .def("setAngle", &Kalman::setAngle)
    .def("getRate", &Kalman::getRate)
    .def("setQangle", &Kalman::setQangle)
    .def("setQbias", &Kalman::setQbias)
    .def("setRmeasure", &Kalman::setRmeasure)
    .def("getQangle", &Kalman::getQangle)
    .def("getQbias", &Kalman::getQbias)
    .def("getRmeasure", &Kalman::getRmeasure);
}
