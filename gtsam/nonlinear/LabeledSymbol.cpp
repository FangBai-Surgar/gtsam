/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LabeledSymbol.h
 * @date Jan 12, 2010
 * @author: Alex Cunningham
 */

#include <iostream>

#include <boost/mpl/char.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/construct.hpp>
#include <boost/lambda/lambda.hpp>

#include <boost/lexical_cast.hpp>

#include <gtsam/nonlinear/LabeledSymbol.h>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
LabeledSymbol::LabeledSymbol()
:  c_(0), label_(0), j_(0) {}

/* ************************************************************************* */
LabeledSymbol::LabeledSymbol(const LabeledSymbol& key)
: c_(key.c_), label_(key.label_), j_(key.j_) {}

/* ************************************************************************* */
LabeledSymbol::LabeledSymbol(unsigned char c, unsigned char label, size_t j)
: c_(c), label_(label), j_(j) {}

/* ************************************************************************* */
LabeledSymbol::LabeledSymbol(gtsam::Key key) {
  const size_t keyBits = sizeof(gtsam::Key) * 8;
  const size_t chrBits = sizeof(unsigned char) * 8;
  const size_t lblBits = sizeof(unsigned char) * 8;
  const size_t indexBits = keyBits - chrBits - lblBits;
  const gtsam::Key chrMask = gtsam::Key(std::numeric_limits<unsigned char>::max()) << (indexBits + lblBits);
  const gtsam::Key lblMask = gtsam::Key(std::numeric_limits<unsigned char>::max()) << indexBits;
  const gtsam::Key indexMask = ~(chrMask | lblMask);
  c_ = (unsigned char)((key & chrMask) >> (indexBits + lblBits));
  label_ = (unsigned char)((key & lblMask) >> indexBits);
  j_ = key & indexMask;
}

/* ************************************************************************* */
LabeledSymbol::operator gtsam::Key() const {
  const size_t keyBits = sizeof(gtsam::Key) * 8;
  const size_t chrBits = sizeof(unsigned char) * 8;
  const size_t lblBits = sizeof(unsigned char) * 8;
  const size_t indexBits = keyBits - chrBits - lblBits;
  const gtsam::Key chrMask = gtsam::Key(std::numeric_limits<unsigned char>::max()) << (indexBits + lblBits);
  const gtsam::Key lblMask = gtsam::Key(std::numeric_limits<unsigned char>::max()) << indexBits;
  const gtsam::Key indexMask = ~(chrMask | lblMask);
  if(j_ > indexMask)
    throw std::invalid_argument("Symbol index is too large");
  gtsam::Key key = (gtsam::Key(c_) << (indexBits + lblBits)) | (gtsam::Key(label_) << indexBits) | j_;
  return key;
}

/* ************************************************************************* */
void LabeledSymbol::print(const std::string& s) const {
  std::cout << s << ": " << (std::string) (*this) << std::endl;
}

/* ************************************************************************* */
LabeledSymbol::operator std::string() const {
  return str(boost::format("%c%c%d") % c_ % label_ % j_);
}

/* ************************************************************************* */
bool LabeledSymbol::operator<(const LabeledSymbol& comp) const {
  return c_ < comp.c_
      || (comp.c_ == c_ && label_ < comp.label_)
      || (comp.c_ == c_ && comp.label_ == label_ && j_ < comp.j_);
}

/* ************************************************************************* */
bool LabeledSymbol::operator==(const LabeledSymbol& comp) const {
  return comp.c_ == c_ && comp.label_ == label_ && comp.j_ == j_;
}

/* ************************************************************************* */
bool LabeledSymbol::operator!=(const LabeledSymbol& comp) const {
  return comp.c_ != c_ || comp.label_ != label_ || comp.j_ != j_;
}

/* ************************************************************************* */
bool LabeledSymbol::operator==(gtsam::Key comp) const {
  return comp == (gtsam::Key)(*this);
}

/* ************************************************************************* */
bool LabeledSymbol::operator!=(gtsam::Key comp) const {
  return comp != (gtsam::Key)(*this);
}

/* ************************************************************************* */
boost::function<bool(gtsam::Key)> LabeledSymbol::TypeTest(unsigned char c) {
  namespace bl = boost::lambda;
  return bl::bind(&LabeledSymbol::chr, bl::bind(bl::constructor<LabeledSymbol>(), bl::_1)) == c;
}

/* ************************************************************************* */
boost::function<bool(gtsam::Key)> LabeledSymbol::LabelTest(unsigned char label) {
  namespace bl = boost::lambda;
  return bl::bind(&LabeledSymbol::label, bl::bind(bl::constructor<LabeledSymbol>(), bl::_1)) == label;
}

/* ************************************************************************* */
boost::function<bool(gtsam::Key)> LabeledSymbol::TypeLabelTest(unsigned char c, unsigned char label) {
  namespace bl = boost::lambda;
  return bl::bind(&LabeledSymbol::chr,   bl::bind(bl::constructor<LabeledSymbol>(), bl::_1)) == c &&
      bl::bind(&LabeledSymbol::label, bl::bind(bl::constructor<LabeledSymbol>(), bl::_1)) == label;
}

/* ************************************************************************* */
std::string _multirobotKeyFormatter(gtsam::Key key) {
  const LabeledSymbol asLabeledSymbol(key);
  if(asLabeledSymbol.chr() > 0 && asLabeledSymbol.label() > 0)
    return (std::string)asLabeledSymbol;

  const gtsam::Symbol asSymbol(key);
  if (asLabeledSymbol.chr() > 0)
    return (std::string)asSymbol;
  else
    return boost::lexical_cast<std::string>(key);
}

/* ************************************************************************* */
void printKeySet(const gtsam::KeySet& keys, const std::string& s, const KeyFormatter& keyFormatter) {
  cout << s << " ";
  if (keys.empty())
    cout << "(none)" << endl;
  else {
    BOOST_FOREACH(const gtsam::Key& key, keys)
        cout << keyFormatter(key) << " ";
    cout << endl;
  }
}

/* ************************************************************************* */
gtsam::KeySet keyIntersection(const gtsam::KeySet& keysA, const gtsam::KeySet& keysB) {
  gtsam::KeySet intersection;
  if (keysA.empty() || keysB.empty())
    return intersection;
  BOOST_FOREACH(const gtsam::Key& key, keysA)
    if (keysB.count(key))
      intersection.insert(key);
  return intersection;
}

/* ************************************************************************* */
bool hasKeyIntersection(const gtsam::KeySet& keysA, const gtsam::KeySet& keysB) {
  if (keysA.empty() || keysB.empty())
    return false;
  BOOST_FOREACH(const gtsam::Key& key, keysA)
    if (keysB.count(key))
      return true;
  return false;
}

/* ************************************************************************* */
gtsam::KeySet keyDifference(const gtsam::KeySet& keysA, const gtsam::KeySet& keysB) {
  if (keysA.empty() || keysB.empty())
    return keysA;

  gtsam::KeySet difference;
  BOOST_FOREACH(const gtsam::Key& key, keysA)
    if (!keysB.count(key))
      difference.insert(key);
  return difference;
}

} // \namespace gtsam

