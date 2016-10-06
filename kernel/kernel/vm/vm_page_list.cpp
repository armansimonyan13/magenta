// Copyright 2016 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <kernel/vm/vm_page_list.h>

#include <err.h>
#include <trace.h>
#include <new.h>
#include <kernel/vm.h>
#include "vm_priv.h"

#define LOCAL_TRACE MAX(VM_GLOBAL_TRACE, 0)

VmPageListNode::VmPageListNode(uint64_t offset)
    : _obj_offset(offset) {
    LTRACEF("%p offset %#" PRIx64 "\n", this, _obj_offset);
}

VmPageListNode::~VmPageListNode() {
    LTRACEF("%p offset %#" PRIx64 "\n", this, _obj_offset);
    DEBUG_ASSERT(_magic == kMagic);

    for (auto p: _pages) {
        DEBUG_ASSERT(p == nullptr);
    }
    _magic = 0;
}

vm_page* VmPageListNode::GetPage(size_t index) {
    DEBUG_ASSERT(_magic == kMagic);
    DEBUG_ASSERT(index < kPageFanOut);
    return _pages[index];
}

status_t VmPageListNode::AddPage(vm_page *p, size_t index) {
    DEBUG_ASSERT(_magic == kMagic);
    DEBUG_ASSERT(index < kPageFanOut);
    if (_pages[index])
        return ERR_ALREADY_EXISTS;
    _pages[index] = p;
    return NO_ERROR;
}

VmPageList::VmPageList() {
    LTRACEF("%p\n", this);
}
VmPageList::~VmPageList() {
    LTRACEF("%p\n", this);
    DEBUG_ASSERT(list_.is_empty());
}

status_t VmPageList::AddPage(vm_page *p, uint64_t offset) {
    uint64_t node_offset = ROUNDDOWN(offset, PAGE_SIZE * VmPageListNode::kPageFanOut);
    auto index = (offset >> PAGE_SIZE_SHIFT) % VmPageListNode::kPageFanOut;

    LTRACEF_LEVEL(2, "%p page %p, offset %#" PRIx64 " node_offset %#" PRIx64 " index %" PRIu64 "\n",
        this, p, offset, node_offset, index);

    // lookup the tree node that holds this page
    auto pln = list_.find(node_offset);
    if (!pln.IsValid()) {
        AllocChecker ac;
        mxtl::unique_ptr<VmPageListNode> pl = mxtl::unique_ptr<VmPageListNode>(new (&ac) VmPageListNode(node_offset));
        if (!ac.check())
            return ERR_NO_MEMORY;

        LTRACEF("allocating new inner node %p\n", pl.get());
        auto status = pl->AddPage(p, index);
        DEBUG_ASSERT(status == NO_ERROR);

        list_.insert(mxtl::move(pl));
    } else {
        pln->AddPage(p, index);
    }

    return NO_ERROR;
}

vm_page* VmPageList::GetPage(uint64_t offset) {
    uint64_t node_offset = ROUNDDOWN(offset, PAGE_SIZE * VmPageListNode::kPageFanOut);
    auto index = (offset >> PAGE_SIZE_SHIFT) % VmPageListNode::kPageFanOut;

    LTRACEF_LEVEL(2, "%p offset %#" PRIx64 " node_offset %#" PRIx64 " index %" PRIu64 "\n",
        this, offset, node_offset, index);

    // lookup the tree node that holds this page
    auto pln = list_.find(node_offset);
    if (!pln.IsValid()) {
        return nullptr;
    }

    return pln->GetPage(index);
}

status_t VmPageList::FreePage(uint64_t offset) {
    LTRACEF_LEVEL(2, "%p offset %#" PRIx64 "\n", this, offset);

    PANIC_UNIMPLEMENTED;

    return NO_ERROR;
}

size_t VmPageList::FreeAllPages() {
    LTRACEF("%p\n", this);;

    list_node list;
    list_initialize(&list);

    size_t count = 0;

    // per page get a reference to the page pointer inside the page list node
    auto per_page_func = [&](vm_page*& p) {
        if (p) {
            // add the page to our list and null out the inner node
            list_add_tail(&list, &p->free.node);
            p = nullptr;
            count++;
        }
    };

    // walk the tree in order, freeing all the pages on every node
    for (auto& pl: list_) {
        // free all of the pages in this node
        pl.ForEveryPage(per_page_func);
    }

    // return all the pages to the pmm at once
    __UNUSED auto freed = pmm_free(&list);
    DEBUG_ASSERT(freed == count);

    // empty the tree
    list_.clear();

    return count;
}
