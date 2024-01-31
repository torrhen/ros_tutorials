#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base {
    // abstract class
    class RegularPolygon {
        public:
            // pluginlib requires empty constructor so parameters of the class are set in initialize()
            virtual void initialize(double side_length) = 0;
            virtual double area() = 0;
            virtual ~RegularPolygon() {}

        protected:
            RegularPolygon() {}
    };
}

#endif
