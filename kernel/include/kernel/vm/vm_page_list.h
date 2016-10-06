// Copyright 2016 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#include <mxtl/unique_ptr.h>
#include <mxtl/intrusive_wavl_tree.h>

struct vm_page;
class VmObject;

class VmPageListNode final : public mxtl::WAVLTreeContainable<mxtl::unique_ptr<VmPageListNode>> {
public:
    VmPageListNode(uint64_t offset);
    ~VmPageListNode();

    static const size_t kPageFanOut = 16;

    uint64_t offset() const { return _obj_offset; }
    uint64_t GetKey() const { return _obj_offset; }

    template <typename T>
    void ForEveryPage(T func) {
        for (auto& p: _pages) {
            func(p);
        }
    }
    template <typename T>
    void ForEveryPage(T func) const {
        for (const auto& p: _pages) {
            func(p);
        }
    }

    vm_page* GetPage(size_t index);
    status_t AddPage(vm_page *p, size_t index);

private:
    static const uint32_t kMagic = 0x504c5354; // 'PLST'
    uint32_t _magic = kMagic;

    uint64_t _obj_offset = 0;
    vm_page* _pages[kPageFanOut] = {};
};

class VmPageList final {
public:
    VmPageList();
    ~VmPageList();

    // walk the page tree, calling the passed in function on every tree node
    template <typename T>
    void ForEveryPage(T per_page_func) {
        for (auto& pl: list_) {
            pl.ForEveryPage(per_page_func);
        }
    }

    status_t AddPage(vm_page *, uint64_t offset);
    vm_page* GetPage(uint64_t offset);
    status_t FreePage(uint64_t offset);
    size_t FreeAllPages();

private:
    mxtl::WAVLTree<uint64_t, mxtl::unique_ptr<VmPageListNode>> list_;
};

