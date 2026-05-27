/*
 * Software Name : networktools
 * SPDX-FileCopyrightText: Copyright (c) Orange SA
 * SPDX-License-Identifier: MIT
 *
 * This software is distributed under the MIT licence,
 * see the "LICENSE" file for more details or https://opensource.org/license/MIT
 *
 * Authors: see CONTRIBUTORS.md
 * Software description: An efficient C++ library for modeling and solving network optimization problems
 */

/**
 * @file
 * @brief Fast NetworkX JSON graph loader with arbitrary properties support.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_RAPID_NETWORKX_IO_H_
#define _NT_RAPID_NETWORKX_IO_H_

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "../../core/string.h"
#include "../../core/arrays.h"
#include "../../core/json.h"
#include "../../core/concepts/digraph.h"
#include "../../graphs/tools.h"
#include "../../core/logging.h"

namespace nt {
  namespace graphs {

    struct Property {
      const char*          key;
      const nt::JsonValue* pv;
      Property() : key(nullptr), pv(nullptr) {}
      Property(const char* key, const nt::JsonValue* pv) : key(key), pv(pv) {}

      bool isNull() const noexcept { return NT_ASSERT_PTR(pv)->IsNull(); }
      bool isFalse() const noexcept { return NT_ASSERT_PTR(pv)->IsFalse(); }
      bool isTrue() const noexcept { return NT_ASSERT_PTR(pv)->IsTrue(); }
      bool isBool() const noexcept { return NT_ASSERT_PTR(pv)->IsBool(); }
      bool isObject() const noexcept { return NT_ASSERT_PTR(pv)->IsObject(); }
      bool isArray() const noexcept { return NT_ASSERT_PTR(pv)->IsArray(); }
      bool isNumber() const noexcept { return NT_ASSERT_PTR(pv)->IsNumber(); }
      bool isInt() const noexcept { return NT_ASSERT_PTR(pv)->IsInt(); }
      bool isUint() const noexcept { return NT_ASSERT_PTR(pv)->IsUint(); }
      bool isInt64() const noexcept { return NT_ASSERT_PTR(pv)->IsInt64(); }
      bool isUint64() const noexcept { return NT_ASSERT_PTR(pv)->IsUint64(); }
      bool isFloat() const noexcept { return NT_ASSERT_PTR(pv)->IsFloat(); }
      bool isDouble() const noexcept { return NT_ASSERT_PTR(pv)->IsDouble(); }
      bool isCstr() const noexcept { return NT_ASSERT_PTR(pv)->IsString(); }

      bool        getBool() const noexcept { return NT_ASSERT_PTR(pv)->GetBool(); }
      int         getInt() const noexcept { return NT_ASSERT_PTR(pv)->GetInt(); }
      unsigned    getUint() const noexcept { return NT_ASSERT_PTR(pv)->GetUint(); }
      int64_t     getInt64() const noexcept { return NT_ASSERT_PTR(pv)->GetInt64(); }
      uint64_t    getUint64() const noexcept { return NT_ASSERT_PTR(pv)->GetUint64(); }
      float       getFloat() const noexcept { return NT_ASSERT_PTR(pv)->GetFloat(); }
      double      getDouble() const noexcept { return NT_ASSERT_PTR(pv)->GetDouble(); }
      const char* getCstr() const noexcept { return NT_ASSERT_PTR(pv)->GetString(); }

      constexpr const nt::JsonValue* getJsonValue() const noexcept { return NT_ASSERT_PTR(pv); }

      BoolDynamicArray getArrayBool() const noexcept {
        const auto&      json_array = NT_ASSERT_PTR(pv)->GetArray();
        BoolDynamicArray array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(v.GetBool());
        return array;
      }

      IntDynamicArray getArrayInt() const noexcept {
        const auto&     json_array = NT_ASSERT_PTR(pv)->GetArray();
        IntDynamicArray array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(v.GetInt());
        return array;
      }

      UintDynamicArray getArrayUint() const noexcept {
        const auto&      json_array = NT_ASSERT_PTR(pv)->GetArray();
        UintDynamicArray array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(v.GetUint());
        return array;
      }

      Int64DynamicArray getArrayInt64() const noexcept {
        const auto&       json_array = NT_ASSERT_PTR(pv)->GetArray();
        Int64DynamicArray array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(v.GetInt64());
        return array;
      }

      Uint64DynamicArray getArrayUint64() const noexcept {
        const auto&        json_array = NT_ASSERT_PTR(pv)->GetArray();
        Uint64DynamicArray array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(v.GetUint64());
        return array;
      }

      FloatDynamicArray getArrayFloat() const noexcept {
        const auto&       json_array = NT_ASSERT_PTR(pv)->GetArray();
        FloatDynamicArray array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(v.GetFloat());
        return array;
      }

      DoubleDynamicArray getArrayDouble() const noexcept {
        const auto&        json_array = NT_ASSERT_PTR(pv)->GetArray();
        DoubleDynamicArray array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(v.GetDouble());
        return array;
      }

      DynamicArray< String > getArrayString() const noexcept {
        const auto&            json_array = NT_ASSERT_PTR(pv)->GetArray();
        DynamicArray< String > array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(String(v.GetString()));
        return array;
      }

      CstrDynamicArray getArrayCstr() const noexcept {
        const auto&      json_array = NT_ASSERT_PTR(pv)->GetArray();
        CstrDynamicArray array;
        array.reserve(json_array.Size());
        for (const auto& v: json_array)
          array.push_back(v.GetString());
        return array;
      }
    };

    template < int MaxProps = 64 >
    struct PropertyBag {
      nt::StaticArray< Property, MaxProps > props;

      bool addProperty(const char* key, const nt::JsonValue* pv) noexcept {
        if (props.full()) return false;
        props.add(Property(key, pv));
        return true;
      }

      const Property* find(const char* key) const noexcept {
        for (int i = 0; i < props.size(); ++i) {
          const Property& p = props[i];
          if (p.key && std::strcmp(p.key, key) == 0) return &p;
        }
        return nullptr;
      }

      bool getBool(const char* key, bool default_val = false) const noexcept {
        const Property* p = find(key);
        if (!p) return default_val;
        return p->getBool();
      }

      int getInt(const char* key, int default_val = 0) const noexcept {
        const Property* p = find(key);
        if (!p) return default_val;
        return p->getInt();
      }

      unsigned getUint(const char* key, unsigned default_val = 0) const noexcept {
        const Property* p = find(key);
        if (!p) return default_val;
        return p->getUint();
      }

      int64_t getInt64(const char* key, int64_t default_val = 0) const noexcept {
        const Property* p = find(key);
        if (!p) return default_val;
        return p->getInt64();
      }

      uint64_t getUint64(const char* key, uint64_t default_val = 0) const noexcept {
        const Property* p = find(key);
        if (!p) return default_val;
        return p->getUint64();
      }

      float getFloat(const char* key, float default_val = 0.0f) const noexcept {
        const Property* p = find(key);
        if (!p) return default_val;
        return p->getFloat();
      }

      double getDouble(const char* key, double default_val = 0.0) const noexcept {
        const Property* p = find(key);
        if (!p) return default_val;
        return p->getDouble();
      }

      const char* getCstr(const char* key, const char* default_val = "") const noexcept {
        const Property* p = find(key);
        if (!p) return default_val;
        return p->getCstr();
      }

      BoolDynamicArray getArrayBool(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return BoolDynamicArray();
        return p->getArrayBool();
      }

      IntDynamicArray getArrayInt(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return IntDynamicArray();
        return p->getArrayInt();
      }

      UintDynamicArray getArrayUint(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return UintDynamicArray();
        return p->getArrayUint();
      }

      Int64DynamicArray getArrayInt64(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return Int64DynamicArray();
        return p->getArrayInt64();
      }

      Uint64DynamicArray getArrayUint64(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return Uint64DynamicArray();
        return p->getArrayUint64();
      }

      FloatDynamicArray getArrayFloat(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return FloatDynamicArray();
        return p->getArrayFloat();
      }

      DoubleDynamicArray getArrayDouble(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return DoubleDynamicArray();
        return p->getArrayDouble();
      }

      DynamicArray< String > getArrayString(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return DynamicArray< String >();
        return p->getArrayString();
      }

      CstrDynamicArray getArrayCstr(const char* key) const noexcept {
        const Property* p = find(key);
        if (!p) return CstrDynamicArray();
        return p->getArrayCstr();
      }

      constexpr int  size() const noexcept { return props.size(); }
      constexpr void removeAll() noexcept { props.removeAll(); }
    };

    namespace detail {
      inline uint64_t fnv1a64(const char* s) {
        constexpr uint64_t FNV_OFFSET = 0xCBF29CE484222325ULL;
        constexpr uint64_t FNV_PRIME = 0x100000001B3ULL;
        uint64_t           h = FNV_OFFSET;
        for (const unsigned char* p = reinterpret_cast< const unsigned char* >(s); *p; ++p) {
          h ^= (uint64_t)(*p);
          h *= FNV_PRIME;
        }
        return h;
      }

      inline uint32_t nextPow2(uint32_t x) {
        if (x <= 1) return 1;
        --x;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        return x + 1;
      }

      struct StringIdMap {
        int    capacity;
        char** keys;
        int*   values;
        StringIdMap() : capacity(0), keys(nullptr), values(nullptr) {}
        ~StringIdMap() { destroy(); }

        bool init(int min_capacity) {
          capacity = (int)nextPow2((uint32_t)min_capacity);
          keys = (char**)NT_MALLOC(sizeof(char*) * capacity);
          values = (int*)NT_MALLOC(sizeof(int) * capacity);
          if (!keys || !values) {
            destroy();
            return false;
          }
          for (int i = 0; i < capacity; ++i) {
            keys[i] = nullptr;
            values[i] = -1;
          }
          return true;
        }

        void destroy() {
          if (keys) NT_FREE(keys);
          if (values) NT_FREE(values);
          keys = nullptr;
          values = nullptr;
          capacity = 0;
        }

        void insert(const char* key, int value) {
          uint64_t h = fnv1a64(key);
          int      mask = capacity - 1, pos = (int)(h & mask);
          while (values[pos] != -1) {
            if (std::strcmp(keys[pos], key) == 0) {
              values[pos] = value;
              return;
            }
            pos = (pos + 1) & mask;
          }
          keys[pos] = const_cast< char* >(key);
          values[pos] = value;
        }

        int find(const char* key) const {
          int      mask = capacity - 1;
          uint64_t h = fnv1a64(key);
          int      pos = (int)(h & mask);
          while (values[pos] != -1) {
            if (std::strcmp(keys[pos], key) == 0) return values[pos];
            pos = (pos + 1) & mask;
          }
          return -1;
        }
      };

      struct IntIdMap {
        int      capacity;
        int64_t* keys;
        int*     values;
        IntIdMap() : capacity(0), keys(nullptr), values(nullptr) {}
        ~IntIdMap() { destroy(); }

        bool init(int min_capacity) {
          capacity = (int)nextPow2((uint32_t)min_capacity);
          keys = (int64_t*)NT_MALLOC(sizeof(int64_t) * capacity);
          values = (int*)NT_MALLOC(sizeof(int) * capacity);
          if (!keys || !values) {
            destroy();
            return false;
          }
          for (int i = 0; i < capacity; ++i) {
            keys[i] = 0;
            values[i] = -1;
          }
          return true;
        }

        void destroy() {
          if (keys) NT_FREE(keys);
          if (values) NT_FREE(values);
          keys = nullptr;
          values = nullptr;
          capacity = 0;
        }

        static inline uint64_t mix(uint64_t x) {
          x ^= x >> 30;
          x *= 0xbf58476d1ce4e5b9ULL;
          x ^= x >> 27;
          x *= 0x94d049bb133111ebULL;
          x ^= x >> 31;
          return x;
        }

        void insert(int64_t key, int value) {
          int mask = capacity - 1, pos = (int)(mix((uint64_t)key) & mask);
          while (values[pos] != -1) {
            if (keys[pos] == key) {
              values[pos] = value;
              return;
            }
            pos = (pos + 1) & mask;
          }
          keys[pos] = key;
          values[pos] = value;
        }

        int find(int64_t key) const {
          int mask = capacity - 1, pos = (int)(mix((uint64_t)key) & mask);
          while (values[pos] != -1) {
            if (keys[pos] == key) return values[pos];
            pos = (pos + 1) & mask;
          }
          return -1;
        }
      };
    }   // namespace detail

    /**
     * @brief NetworkX JSON graph loader
     *
     * @code
     * RapidNetworkxIo<SmartDigraph> loader;
     * SmartDigraph g;
     * if (loader.readFile("graph.json", g)) {
     *   // Access properties
     *   for (SmartDigraph::NodeIt n(g); n != INVALID; ++n) {
     *     double weight = loader.nodeProperties()[n].getDouble("weight", 1.0);
     *   }
     * }
     *
     * // Customization
     * loader.setSourceAttr("from");
     * loader.setTargetAttr("to");
     * @endcode
     *
     * @tparam GR
     * @tparam MaxNodeProps
     * @tparam MaxArcProps
     * @tparam MaxGraphProps
     */
    template < typename GR, int MaxNodeProps = 64, int MaxArcProps = 64, int MaxGraphProps = 16 >
    struct RapidNetworkxIo {
      using Graph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Graph);
      using GraphPropBag = PropertyBag< MaxGraphProps >;
      using NodePropBag = PropertyBag< MaxNodeProps >;
      using ArcPropBag = PropertyBag< MaxArcProps >;
      using NodePropMap = typename Graph::template StaticNodeMap< NodePropBag >;
      using ArcPropMap = typename Graph::template StaticArcMap< ArcPropBag >;

      using StringBuffer = rapidjson::StringBuffer;
      using JsonWriter = rapidjson::Writer< StringBuffer >;

      char*            _buffer;
      size_t           _buffer_size;
      nt::JSONDocument _doc;
      NodePropMap      _node_props;
      ArcPropMap       _arc_props;
      GraphPropBag     _graph_props;
      nt::String       _source_attr, _target_attr, _id_attr, _links_attr;

      // Cached objects for JSON serialization (mutable for const methods)
      // Note: _json_buffer is mutable and used for serialization cache,
      // no need to move it as it will be regenerated when needed
      mutable StringBuffer _json_buffer;

      /**
       * @brief Construct a new RapidNetworkxIo Loader object
       *
       */
      RapidNetworkxIo() :
          _buffer(nullptr), _buffer_size(0), _source_attr("source"), _target_attr("target"), _id_attr("id"),
          _links_attr("links") {}

      /**
       * @brief Constructs a RapidNetworkxIo object with attribute names.
       *
       * @param sz_source_attr The attribute name for the source of the links.
       * @param sz_target_attr The attribute name for the target of the links.
       */
      RapidNetworkxIo(const char* sz_source_attr, const char* sz_target_attr) noexcept :
          _buffer(nullptr), _buffer_size(0), _source_attr(sz_source_attr), _target_attr(sz_target_attr), _id_attr("id"),
          _links_attr("links") {}

      /**
       * @brief Constructs a RapidNetworkxIo object with attribute names.
       *
       * @param sz_source_attr The attribute name for the source of the links.
       * @param sz_target_attr The attribute name for the target of the links.
       * @param sz_name_attr The attribute name for the node IDs.
       * @param sz_key_attr The attribute name for the link keys.
       * @param sz_link_attr The attribute name for the links.
       */
      RapidNetworkxIo(const char* sz_source_attr,
                      const char* sz_target_attr,
                      const char* sz_name_attr,
                      const char* sz_key_attr,
                      const char* sz_link_attr) noexcept :
          _buffer(nullptr),
          _buffer_size(0), _source_attr(sz_source_attr), _target_attr(sz_target_attr), _id_attr(sz_name_attr),
          _links_attr(sz_link_attr) {}

      /**
       * @brief Destroy the Networkx Loader object
       *
       */
      ~RapidNetworkxIo() {
        if (_buffer) {
          NT_FREE(_buffer);
          _buffer = nullptr;
        }
      }

      RapidNetworkxIo(const RapidNetworkxIo&) = delete;
      RapidNetworkxIo& operator=(const RapidNetworkxIo&) = delete;

      void                          setSourceAttr(const char* attr) noexcept { _source_attr.assign(attr); }
      void                          setTargetAttr(const char* attr) noexcept { _target_attr.assign(attr); }
      void                          setIdAttr(const char* attr) noexcept { _id_attr.assign(attr); }
      void                          setLinksAttr(const char* attr) noexcept { _links_attr.assign(attr); }
      constexpr const GraphPropBag& graphProperties() const noexcept { return _graph_props; }
      constexpr const NodePropMap&  nodeProperties() const noexcept { return _node_props; }
      constexpr const ArcPropMap&   arcProperties() const noexcept { return _arc_props; }

      /**
       * @brief Extract a node map of int8_t from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of int8_t
       *
       */
      ByteNodeMap getNodeMapByte(const Graph& g, const char* attr, std::int8_t default_val = 0) const noexcept {
        ByteNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = (std::int8_t)_node_props[n].getInt(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of doubles from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of doubles
       */
      BoolNodeMap getNodeMapBool(const Graph& g, const char* attr, bool default_val = false) const noexcept {
        BoolNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = _node_props[n].getBool(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of integers from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of int
       */
      IntNodeMap getNodeMapInt(const Graph& g, const char* attr, std::int32_t default_val = 0) const noexcept {
        IntNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = (std::int32_t)_node_props[n].getInt(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of unsigned integers from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of unsigned int
       */
      UintNodeMap getNodeMapUint(const Graph& g, const char* attr, std::uint32_t default_val = 0) const noexcept {
        UintNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = (std::uint32_t)_node_props[n].getUint(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of int64_t from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of int64_t
       */
      Int64NodeMap getNodeMapInt64(const Graph& g, const char* attr, std::int64_t default_val = 0) const noexcept {
        Int64NodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = _node_props[n].getInt64(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of uint64_t from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of uint64_t
       */
      Uint64NodeMap getNodeMapUint64(const Graph& g, const char* attr, std::uint64_t default_val = 0) const noexcept {
        Uint64NodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = _node_props[n].getUint64(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of floats from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of floats
       */
      FloatNodeMap getNodeMapFloat(const Graph& g, const char* attr, float default_val = 0.0f) const noexcept {
        FloatNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = _node_props[n].getFloat(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of doubles from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of doubles
       */
      DoubleNodeMap getNodeMapDouble(const Graph& g, const char* attr, double default_val = 0.0) const noexcept {
        DoubleNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = _node_props[n].getDouble(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of C-strings from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of const char*
       */
      CstrNodeMap getNodeMapCstr(const Graph& g, const char* attr, const char* default_val = "") const noexcept {
        CstrNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = _node_props[n].getCstr(attr, default_val);
        return map;
      }

      /**
       * @brief Extract a node map of strings from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of nt::String
       */
      StringNodeMap getNodeMapString(const Graph& g, const char* attr, const char* default_val = "") const noexcept {
        StringNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = String(_node_props[n].getCstr(attr, default_val));
        return map;
      }

      /**
       * @brief Extract a node map of properties from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @return NodeMap of const Property*
       */
      typename Graph::template NodeMap< const Property* > getNodeMapProperty(const Graph& g,
                                                                             const char*  attr) const noexcept {
        typename Graph::template NodeMap< const Property* > map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n)
          map[n] = _node_props[n].find(attr);
        return map;
      }

      /**
       * @brief Extract a node map of a property as a JSON-string
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return NodeMap of nt::String
       */
      StringNodeMap
         getNodeMapPropertyAsString(const Graph& g, const char* attr, const char* default_val = "") const noexcept {
        StringNodeMap map(g);
        if (_node_props.empty()) return map;
        for (typename Graph::NodeIt n(g); n != INVALID; ++n) {
          const Property* p = _node_props[n].find(attr);
          if (!p)
            map[n] = default_val;
          else
            map[n] = std::move(propertyToString(*p));
        }
        return map;
      }

      /**
       * @brief Extract an arc map of int8_t from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of int8_t
       *
       */
      ByteArcMap getArcMapByte(const Graph& g, const char* attr, std::int8_t default_val = 0) const noexcept {
        ByteArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = (std::int8_t)_arc_props[a].getInt(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of doubles from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of doubles
       */
      BoolArcMap getArcMapBool(const Graph& g, const char* attr, bool default_val = false) const noexcept {
        BoolArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = _arc_props[a].getBool(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of integers from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of int
       */
      IntArcMap getArcMapInt(const Graph& g, const char* attr, std::int32_t default_val = 0) const noexcept {
        IntArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = (std::int32_t)(*_arc_props)[a].getInt(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of unsigned integers from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of unsigned int
       *
       */
      UintArcMap getArcMapUint(const Graph& g, const char* attr, std::uint32_t default_val = 0) const noexcept {
        UintArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = (std::uint32_t)(*_arc_props)[a].getUint(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of int64_t from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of int64_t
       */
      Int64ArcMap getArcMapInt64(const Graph& g, const char* attr, std::int64_t default_val = 0) const noexcept {
        Int64ArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = _arc_props[a].getInt64(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of uint64_t from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of uint64_t
       */
      Uint64ArcMap getArcMapUint64(const Graph& g, const char* attr, std::uint64_t default_val = 0) const noexcept {
        Uint64ArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = _arc_props[a].getUint64(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of floats from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of floats
       */
      FloatArcMap getArcMapFloat(const Graph& g, const char* attr, float default_val = 0.0f) const noexcept {
        FloatArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = _arc_props[a].getFloat(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of doubles from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of doubles
       */
      DoubleArcMap getArcMapDouble(const Graph& g, const char* attr, double default_val = 0.0) const noexcept {
        DoubleArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = _arc_props[a].getDouble(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of strings from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of nt::String
       */
      StringArcMap getArcMapString(const Graph& g, const char* attr, const char* default_val = "") const noexcept {
        StringArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = String(_arc_props[a].getCstr(attr, default_val));
        return map;
      }

      /**
       * @brief Extract an arc map of C-strings from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of const char*
       */
      CstrArcMap getArcMapCstr(const Graph& g, const char* attr, const char* default_val = "") const noexcept {
        CstrArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = _arc_props[a].getCstr(attr, default_val);
        return map;
      }

      /**
       * @brief Extract an arc map of properties from a property attribute
       *
       * @param g The graph
       * @param attr The property attribute name
       * @return ArcMap of const Property*
       */
      typename Graph::template ArcMap< const Property* > getArcMapProperty(const Graph& g,
                                                                           const char*  attr) const noexcept {
        typename Graph::template ArcMap< const Property* > map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a)
          map[a] = _arc_props[a].find(attr);
        return map;
      }

      /**
       * @brief Extract an arc map of a property as a JSON-string
       *
       * @param g The graph
       * @param attr The property attribute name
       * @param default_val Default value for missing properties
       * @return ArcMap of nt::String
       */
      StringArcMap
         getArcMapPropertyAsString(const Graph& g, const char* attr, const char* default_val = "") const noexcept {
        StringArcMap map(g);
        if (_arc_props.empty()) return map;
        for (typename Graph::ArcIt a(g); a != INVALID; ++a) {
          const Property* p = _arc_props[a].find(attr);
          if (!p)
            map[a] = default_val;
          else
            map[a] = std::move(propertyToString(*p));
        }
        return map;
      }

      /**
       * @brief Get a JSON object or array as a string representation
       *
       * @param property The property to look up
       * @param buffer Output buffer to write the JSON string (will be null-terminated)
       * @param buffer_size Size of the output buffer
       * @return Pointer to buffer on success, default_val on failure
       *
       * @note The buffer must be large enough to hold the JSON string, or the result will be truncated
       * @note Thread-safe only if each PropertyBag instance is accessed by a single thread
       */
      const char* propertyToString(const Property& property, char* buffer, size_t buffer_size) const noexcept {
        if (!buffer || buffer_size == 0) return nullptr;

        _json_buffer.Clear();
        JsonWriter writer(_json_buffer);
        NT_ASSERT_PTR(property.pv)->Accept(writer);

        const char*  json_str = _json_buffer.GetString();
        const size_t json_len = _json_buffer.GetSize();

        nt::strcpy_safe(buffer, json_str, buffer_size);
        return buffer;
      }

      /**
       * @brief Get a JSON object or array as a dynamically allocated string
       *
       * @param property The property to look up
       * @return Allocated string (caller must free with std::free), or nullptr if not found or not an object/array
       *
       * @note Thread-safe only if each PropertyBag instance is accessed by a single thread
       */
      nt::String propertyToString(const Property& property) const noexcept {
        _json_buffer.Clear();
        JsonWriter writer(_json_buffer);
        NT_ASSERT_PTR(property.pv)->Accept(writer);

        const char*  json_str = _json_buffer.GetString();
        const size_t json_len = _json_buffer.GetSize();

        return nt::String(json_str, json_len);
      }

      /**
       * @brief Read a graph directly from an already-parsed JSON value.
       *
       * @param value A JSON object. The caller must keep the owning document alive for as long as node/arc
       *              property maps are accessed, since Property pointers reference values
       *              inside @p value.
       * @param g The graph to populate.
       * @return true if successfully read, false otherwise.
       */
      bool read(const nt::JsonValue& value, Graph& g) noexcept {
        if (!value.IsObject()) return false;
        return _loadFromValue(value, g);
      }

      /**
       * @brief Read a graph from a file
       *
       * @param filename The name of the file to read
       * @param g The graph to populate
       * @return true if the file was successfully read and parsed, false otherwise
       */
      bool readFile(const char* filename, Graph& g) noexcept {
        if (_buffer) {
          NT_FREE(_buffer);
          _buffer = nullptr;
        }

        FILE* f = std::fopen(filename, "rb");
        if (!f) return false;
        if (std::fseek(f, 0, SEEK_END) != 0) {
          std::fclose(f);
          return false;
        }
        long sz = std::ftell(f);
        if (sz < 0) {
          std::fclose(f);
          return false;
        }
        if (std::fseek(f, 0, SEEK_SET) != 0) {
          std::fclose(f);
          return false;
        }
        _buffer = (char*)NT_MALLOC((size_t)sz + 1);
        if (!_buffer) {
          std::fclose(f);
          return false;
        }
        size_t rd = std::fread(_buffer, 1, (size_t)sz, f);
        std::fclose(f);
        if (rd != (size_t)sz) {
          NT_FREE(_buffer);
          _buffer = nullptr;
          return false;
        }
        _buffer[sz] = '\0';
        _buffer_size = (size_t)sz + 1;
        return _parseAndLoad(g);
      }

      // private
      bool _parseAndLoad(Graph& g) noexcept {
        _doc.ParseInsitu(_buffer);
        if (_doc.HasParseError() || !_doc.IsObject()) return false;
        return _loadFromValue(_doc, g);
      }

      bool _loadFromValue(const nt::JsonValue& root, Graph& g) noexcept {
        // Get graph properties
        const nt::JsonValue* p_nodes = nullptr;
        const nt::JsonValue* p_edges = nullptr;
        for (nt::JsonObject::ConstMemberIterator it = root.MemberBegin(); it != root.MemberEnd(); ++it) {
          const char* sz_name = it->name.GetString();
          if (std::strcmp(sz_name, "nodes") == 0 && it->value.IsArray()) {
            p_nodes = &it->value;
          } else if (std::strcmp(sz_name, _links_attr.c_str()) == 0 && it->value.IsArray()) {
            p_edges = &it->value;
          } else {
            _graph_props.addProperty(sz_name, &it->value);
          }
        }

        if (!p_nodes) {
          LOG_F(ERROR, "Missing 'nodes' array");
          return false;
        }
        const nt::JsonConstArray nodes = p_nodes->GetArray();
        const int                node_count = (int)nodes.Size();
        _node_props.build(node_count);

        if (!p_edges) {
          LOG_F(ERROR, "Missing '{}' array", _links_attr.c_str());
          return false;
        }
        const nt::JsonConstArray edges = p_edges->GetArray();
        const int                edge_count = (int)edges.Size();
        _arc_props.build(edge_count);

        detail::StringIdMap str_map;
        detail::IntIdMap    int_map;
        if (!str_map.init(node_count * 2) || !int_map.init(node_count * 2)) return false;

        nt::TrivialDynamicArray< Node > nodeArray;
        nodeArray.reserve(node_count);

        for (int i = 0; i < node_count; ++i) {
          const nt::JsonValue& n = nodes[(nt::JsonSizeType)i];
          if (!n.IsObject()) return false;

          const nt::JsonObject::ConstMemberIterator p_member = n.FindMember(_id_attr.c_str());
          if (p_member == n.MemberEnd()) {
            LOG_F(ERROR, "Missing attribute '{}'", _id_attr.c_str());
            return false;
          }

          if (p_member->value.IsString())
            str_map.insert(p_member->value.GetString(), i);
          else if (p_member->value.IsInt64())
            int_map.insert(p_member->value.GetInt64(), i);
          else if (p_member->value.IsUint64())
            int_map.insert((int64_t)p_member->value.GetUint64(), i);
          else
            return false;

          Node node;
          if constexpr (nt::concepts::BuildTagIndicator< Graph >) {
            node = Node(i);
          } else {
            node = g.addNode();
            nodeArray.addEmplace(node);
          }

          _fillPropertyBag(n, _node_props[node], nullptr, 0);
        }

        nt::DynamicArray< nt::IntDynamicArray > arcs;
        struct EdgeMapping {
          int src_idx;
          int tgt_idx;
          int edge_json_idx;
        };
        nt::TrivialDynamicArray< EdgeMapping > edge_mappings;

        if constexpr (nt::concepts::BuildTagIndicator< Graph >) {
          arcs.ensureAndFill(node_count);
          edge_mappings.reserve(edge_count);
        }

        const char* exclude_keys[] = {_source_attr.c_str(), _target_attr.c_str()};

        for (int i = 0; i < edge_count; ++i) {
          const nt::JsonValue& e = edges[(nt::JsonSizeType)i];
          NT_ASSERT(e.IsObject(), "Edge is not a JSON object");

          const nt::JsonObject::ConstMemberIterator p_source = e.FindMember(_source_attr.c_str());
          if (p_source == e.MemberEnd()) {
            LOG_F(ERROR, "Missing attribute '{}'", _source_attr.c_str());
            return false;
          }

          const nt::JsonObject::ConstMemberIterator p_target = e.FindMember(_target_attr.c_str());
          if (p_target == e.MemberEnd()) {
            LOG_F(ERROR, "Missing attribute '{}'", _target_attr.c_str());
            return false;
          }

          const nt::JsonValue& src_val = p_source->value;

          int src_idx = -1;
          if (src_val.IsString())
            src_idx = str_map.find(src_val.GetString());
          else if (src_val.IsInt64())
            src_idx = int_map.find(src_val.GetInt64());
          else if (src_val.IsUint64())
            src_idx = int_map.find((int64_t)src_val.GetUint64());

          const nt::JsonValue& tgt_val = p_target->value;

          int tgt_idx = -1;
          if (tgt_val.IsString())
            tgt_idx = str_map.find(tgt_val.GetString());
          else if (tgt_val.IsInt64())
            tgt_idx = int_map.find(tgt_val.GetInt64());
          else if (tgt_val.IsUint64())
            tgt_idx = int_map.find((int64_t)tgt_val.GetUint64());

          if (src_idx < 0 || tgt_idx < 0) {
            LOG_F(WARNING,
                  "Source or target node not found for edge {}: source '{}', target '{}'",
                  i,
                  src_val.GetString(),
                  tgt_val.GetString());
            continue;
          }

          if constexpr (nt::concepts::BuildTagIndicator< Graph >) {
            arcs[src_idx].add(tgt_idx);
            edge_mappings.add({src_idx, tgt_idx, i});
          } else {
            const Arc arc = g.addArc(nodeArray[src_idx], nodeArray[tgt_idx]);
            _fillPropertyBag(e, _arc_props[arc], exclude_keys, 2);
          }
        }

        if constexpr (nt::concepts::BuildTagIndicator< Graph >) {
          g.build(edge_mappings.size(), arcs);

          // Now fill arc properties by finding each arc
          for (const EdgeMapping& mapping: edge_mappings) {
            const nt::JsonValue& e = edges[(nt::JsonSizeType)mapping.edge_json_idx];
            NT_ASSERT(e.IsObject(), "Edge is not a JSON object");

            const Node src_node = Node(mapping.src_idx);
            const Node tgt_node = Node(mapping.tgt_idx);

            Arc arc = findArc(g, src_node, tgt_node);
            NT_ASSERT(arc != INVALID, "Arc not found");
            _fillPropertyBag(e, _arc_props[arc], exclude_keys, 2);
          }
        }

        return true;
      }

      template < int MaxProps >
      void _fillPropertyBag(const nt::JsonValue&     obj,
                            PropertyBag< MaxProps >& bag,
                            const char**             exclude_keys,
                            int                      excludeCount) noexcept {
        if (!obj.IsObject()) return;
        for (auto m = obj.MemberBegin(); m != obj.MemberEnd(); ++m) {
          const char* key = m->name.GetString();
          bool        excluded = false;
          for (int i = 0; i < excludeCount; ++i) {
            if (exclude_keys[i] && std::strcmp(key, exclude_keys[i]) == 0) {
              excluded = true;
              break;
            }
          }
          if (excluded || bag.size() >= MaxProps) continue;
          bag.addProperty(key, &m->value);
        }
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif   // _NT_NETWORKX_H_
