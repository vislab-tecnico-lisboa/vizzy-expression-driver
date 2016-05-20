#ifndef EXPRESSION_VALS_HPP
#define EXPRESSION_VALS_HPP

#include <string>
#include <boost/asio.hpp>


namespace ExpressionValues
{

class ExpressionNeutral{

public:
    static const uint8_t LEFT_EYEBROW;
    static const uint8_t RIGHT_EYEBROW;
    static const uint8_t MOUTH;
    static const uint8_t EYELIDS;
};


class ExpressionHappy{

public:
    static const uint8_t LEFT_EYEBROW;
    static const uint8_t RIGHT_EYEBROW;
    static const uint8_t MOUTH;
    static const uint8_t EYELIDS;
};

class ExpressionSad{

public:
    static const uint8_t LEFT_EYEBROW;
    static const uint8_t RIGHT_EYEBROW;
    static const uint8_t MOUTH;
    static const uint8_t EYELIDS;
};

class ExpressionSurprised{

public:
    static const uint8_t LEFT_EYEBROW;
    static const uint8_t RIGHT_EYEBROW;
    static const uint8_t MOUTH;
    static const uint8_t EYELIDS;
};

class ExpressionAngry{

public:
    static const uint8_t LEFT_EYEBROW;
    static const uint8_t RIGHT_EYEBROW;
    static const uint8_t MOUTH;
    static const uint8_t EYELIDS;
};

class ExpressionEvil{

public:
    static const uint8_t LEFT_EYEBROW;
    static const uint8_t RIGHT_EYEBROW;
    static const uint8_t MOUTH;
    static const uint8_t EYELIDS;
};

class ExpressionShy{

public:
    static const uint8_t LEFT_EYEBROW;
    static const uint8_t RIGHT_EYEBROW;
    static const uint8_t MOUTH;
    static const uint8_t EYELIDS;
};

class ExpressionCunning{

public:
    static const uint8_t LEFT_EYEBROW;
    static const uint8_t RIGHT_EYEBROW;
    static const uint8_t MOUTH;
    static const uint8_t EYELIDS;
};

uint8_t ExpressionValue(std::string emotion, std::string part);

}

#endif // EXPRESSION_VALS_HPP
