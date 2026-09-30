#pragma once

#include <RE/Skyrim.h>
#include <SKSE/SKSE.h>
#include "RuntimeSupport.h"
#include <array>
#include <cmath>
#include <cstdint>

namespace fc {

class NativeShapeWitness {
public:

    static bool LogLocked(const RE::hkpShape* root, float gameUnitsPerHavokUnit,
                          const char* reason, unsigned attachmentsSinceLoad,
                          const RE::hkTransform* phantomTransform=nullptr) {

        if(!runtime::supported() || !root ||
           !std::isfinite(gameUnitsPerHavokUnit) || gameUnitsPerHavokUnit<=0 ||
           gameUnitsPerHavokUnit>1000)return false;
        NativeShapeWitness witness(gameUnitsPerHavokUnit);
        SKSE::log::info("Native shape witness: reason={} attachmentsSinceLoad={} root={} scale={:.6f}; read-only, local bounds include convex skin",
            reason?reason:"unspecified",attachmentsSinceLoad,static_cast<const void*>(root),gameUnitsPerHavokUnit);
        if(phantomTransform) {
            const auto x=components(phantomTransform->rotation.col0);
            const auto y=components(phantomTransform->rotation.col1);
            const auto z=components(phantomTransform->rotation.col2);
            const auto p=components(phantomTransform->translation);
            SKSE::log::info("Native shape phantom: columnX=({:.4f},{:.4f},{:.4f}) columnY=({:.4f},{:.4f},{:.4f}) columnZ=({:.4f},{:.4f},{:.4f}) translation=({:.2f},{:.2f},{:.2f})",
                x[0],x[1],x[2],y[0],y[1],y[2],z[0],z[1],z[2],
                p[0]*witness.scale_,p[1]*witness.scale_,p[2]*witness.scale_);
        }
        witness.visit(root,0,-1,RE::HK_INVALID_SHAPE_KEY,true);
        SKSE::log::info("Native shape witness end: nodes={} truncated={}",witness.count_,witness.truncated_);
        return witness.count_!=0;
    }

private:
    static constexpr unsigned maxNodes=12;
    static constexpr unsigned maxDepth=3;
    float scale_;
    unsigned count_{};
    bool truncated_{};
    std::array<const RE::hkpShape*,maxDepth+1> ancestors_{};

    explicit NativeShapeWitness(float scale):scale_(scale){}

    static std::array<float,4> components(const RE::hkVector4& value) {
        std::array<float,4> result{};
        _mm_storeu_ps(result.data(),value.quad);
        return result;
    }
    static const char* typeName(RE::hkpShapeType type) {
        switch(type) {
        case RE::hkpShapeType::kSphere:return "sphere";
        case RE::hkpShapeType::kBox:return "box";
        case RE::hkpShapeType::kCapsule:return "capsule";
        case RE::hkpShapeType::kConvexVertices:return "convex-vertices";
        case RE::hkpShapeType::kList:return "list";
        case RE::hkpShapeType::kConvexTranslate:return "convex-translate";
        case RE::hkpShapeType::kConvexTransform:return "convex-transform";
        case RE::hkpShapeType::kTransform:return "transform";
        default:return "uninspected";
        }
    }
    static bool simpleConvex(RE::hkpShapeType type) {
        return type==RE::hkpShapeType::kSphere || type==RE::hkpShapeType::kBox ||
            type==RE::hkpShapeType::kCapsule || type==RE::hkpShapeType::kConvexVertices;
    }
    static bool singleWrapper(RE::hkpShapeType type) {
        return type==RE::hkpShapeType::kConvexTranslate ||
            type==RE::hkpShapeType::kConvexTransform || type==RE::hkpShapeType::kTransform;
    }
    void bounds(unsigned node,const RE::hkpShape* shape,const char* source="native-leaf") const {

        RE::hkTransform identity{};
        identity.rotation.col0=RE::hkVector4(1,0,0,0);
        identity.rotation.col1=RE::hkVector4(0,1,0,0);
        identity.rotation.col2=RE::hkVector4(0,0,1,0);
        RE::hkAabb aabb{};
        shape->GetAabbImpl(identity,0,aabb);
        logBounds(node,source,components(aabb.min),components(aabb.max));
    }
    void logBounds(unsigned node,const char* source,const std::array<float,4>& minimum,
                   const std::array<float,4>& maximum) const {
        bool valid=true;
        for(unsigned axis=0;axis<3;++axis)
            valid=valid&&std::isfinite(minimum[axis])&&std::isfinite(maximum[axis])&&minimum[axis]<=maximum[axis];
        SKSE::log::info("Native shape bounds: node={} source={} valid={} min=({:.2f},{:.2f},{:.2f}) max=({:.2f},{:.2f},{:.2f})",
            node,source,valid,minimum[0]*scale_,minimum[1]*scale_,minimum[2]*scale_,
            maximum[0]*scale_,maximum[1]*scale_,maximum[2]*scale_);
    }
    void visit(const RE::hkpShape* shape,unsigned depth,int parent,RE::hkpShapeKey key,bool enabled) {
        if(!shape)return;
        if(depth>maxDepth || count_>=maxNodes){truncated_=true;return;}
        for(unsigned i=0;i<depth;++i)if(ancestors_[i]==shape){truncated_=true;return;}
        ancestors_[depth]=shape;
        const unsigned node=count_++;
        const auto type=shape->type;
        SKSE::log::info("Native shape node: node={} parent={} key={} depth={} enabled={} pointer={} type={}({})",
            node,parent,key,depth,enabled,static_cast<const void*>(shape),static_cast<int>(type),typeName(type));
        if(simpleConvex(type)) {
            const auto* convex=static_cast<const RE::hkpConvexShape*>(shape);
            SKSE::log::info("Native shape convex skin: node={} radius={:.3f}",node,convex->radius*scale_);
            if(type==RE::hkpShapeType::kCapsule) {
                const auto* capsule=static_cast<const RE::hkpCapsuleShape*>(shape);
                const auto a=components(capsule->vertexA),b=components(capsule->vertexB);
                SKSE::log::info("Native shape capsule: node={} A=({:.2f},{:.2f},{:.2f}) B=({:.2f},{:.2f},{:.2f}) radius={:.3f}",
                    node,a[0]*scale_,a[1]*scale_,a[2]*scale_,b[0]*scale_,b[1]*scale_,b[2]*scale_,capsule->radius*scale_);
            }
            bounds(node,shape);
        } else if(type==RE::hkpShapeType::kList) {
            const auto* list=static_cast<const RE::hkpListShape*>(shape);
            const auto size=list->childInfo.size();
            SKSE::log::info("Native shape list: node={} children={} disabled={}",node,size,list->numDisabledChildren);
            const auto centre=components(list->aabbCenter),half=components(list->aabbHalfExtents);
            std::array<float,4> minimum{},maximum{};
            for(unsigned axis=0;axis<3;++axis){minimum[axis]=centre[axis]-half[axis];maximum[axis]=centre[axis]+half[axis];}
            logBounds(node,"cached-list",minimum,maximum);

            if(size<0 || size>256 || size>list->childInfo.capacity() || (size&&!list->childInfo.data())){
                truncated_=true;return;
            }
            for(int i=0;i<size;++i) {
                if(count_>=maxNodes || depth>=maxDepth){truncated_=true;break;}
                const bool childEnabled=(list->enabledChildren[static_cast<unsigned>(i)/32]&(1U<<(static_cast<unsigned>(i)%32)))!=0;
                SKSE::log::info("Native shape child: parent={} key={} filter={:08X} enabled={}",
                    node,i,list->childInfo[i].collisionFilterInfo.filter,childEnabled);
                visit(list->childInfo[i].shape,depth+1,static_cast<int>(node),static_cast<RE::hkpShapeKey>(i),childEnabled);
            }
        } else if(singleWrapper(type)) {

            const auto* container=shape->GetContainer();
            if(!container)return;
            const auto children=container->GetNumChildShapes();
            SKSE::log::info("Native shape wrapper: node={} children={} child coordinates are wrapper-local",node,children);
            if(children!=1 || depth>=maxDepth || count_>=maxNodes){truncated_=true;return;}
            const auto childKey=container->GetFirstKey();
            if(childKey==RE::HK_INVALID_SHAPE_KEY){truncated_=true;return;}
            RE::hkpShapeBuffer buffer{};
            const auto* child=container->GetChildShape(childKey,buffer);

            if(child && simpleConvex(child->type))bounds(node,shape,"native-wrapper-leaf");
            visit(child,depth+1,static_cast<int>(node),childKey,true);
        }
    }
};

}
