// Copyright (C) 2023 Christian Mazakas
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_UNORDERED_DETAIL_FOA_NODE_MAP_TYPES_HPP
#define BOOST_UNORDERED_DETAIL_FOA_NODE_MAP_TYPES_HPP

#include <boost/core/allocator_access.hpp>
#include <boost/core/no_exceptions_support.hpp>
#include <boost/core/pointer_traits.hpp>

namespace boost {
  namespace unordered {
    namespace detail {
      namespace foa {
        template <class Key, class T> struct node_map_types
        {
          using key_type = Key;
          using mapped_type = T;
          using raw_key_type = typename std::remove_const<Key>::type;
          using raw_mapped_type = typename std::remove_const<T>::type;

          using init_type = std::pair<raw_key_type, raw_mapped_type>;
          using value_type = std::pair<Key const, T>;
          using moved_type = std::pair<raw_key_type&&, raw_mapped_type&&>;

          using element_type = foa::element_type<value_type>;

          static value_type& value_from(element_type const& x)
          {
            return *(x.p);
          }

          template <class K, class V>
          static raw_key_type const& extract(std::pair<K, V> const& kv)
          {
            return kv.first;
          }

          static raw_key_type const& extract(element_type const& kv)
          {
            return kv.p->first;
          }

          static element_type&& move(element_type& x) { return std::move(x); }
          static moved_type move(init_type& x)
          {
            return {std::move(x.first), std::move(x.second)};
          }

          static moved_type move(value_type& x)
          {
            return {std::move(const_cast<raw_key_type&>(x.first)),
              std::move(const_cast<raw_mapped_type&>(x.second))};
          }

          template <class A>
          static void construct(A&, element_type* p, element_type&& x) noexcept
          {
            p->p = x.p;
            x.p = nullptr;
          }

          template <class A>
          static void construct(
            A& al, element_type* p, element_type const& copy)
          {
            construct(al, p, *copy.p);
          }

          template <class A, class... Args>
          static void construct(A& al, init_type* p, Args&&... args)
          {
            boost::allocator_construct(al, p, std::forward<Args>(args)...);
          }

          template <class A, class... Args>
          static void construct(A& al, value_type* p, Args&&... args)
          {
            boost::allocator_construct(al, p, std::forward<Args>(args)...);
          }

          template <class A, class... Args>
          static void construct(A& al, element_type* p, Args&&... args)
          {
            p->p = boost::to_address(boost::allocator_allocate(al, 1));
            BOOST_TRY
            {
              boost::allocator_construct(al, p->p, std::forward<Args>(args)...);
            }
            BOOST_CATCH(...)
            {
              using pointer_type = typename boost::allocator_pointer<A>::type;
              using pointer_traits = boost::pointer_traits<pointer_type>;

              boost::allocator_deallocate(
                al, pointer_traits::pointer_to(*(p->p)), 1);
              BOOST_RETHROW
            }
            BOOST_CATCH_END
          }

          template <class A> static void destroy(A& al, value_type* p) noexcept
          {
            boost::allocator_destroy(al, p);
          }

          template <class A> static void destroy(A& al, init_type* p) noexcept
          {
            boost::allocator_destroy(al, p);
          }

          template <class A>
          static void destroy(A& al, element_type* p) noexcept
          {
            if (p->p) {
              using pointer_type = typename boost::allocator_pointer<A>::type;
              using pointer_traits = boost::pointer_traits<pointer_type>;

              destroy(al, p->p);
              boost::allocator_deallocate(
                al, pointer_traits::pointer_to(*(p->p)), 1);
            }
          }
        };

      } // namespace foa
    }   // namespace detail
  }     // namespace unordered
} // namespace boost

#endif // BOOST_UNORDERED_DETAIL_FOA_NODE_MAP_TYPES_HPP
