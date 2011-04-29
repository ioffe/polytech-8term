#pragma once

namespace cg
{
namespace verification
{

// Specify which contacts or overlaps are treated as intersections.
// Contact here means that two non-adjacent edges has intersection in one of these edges ends.
// In further code contact called overlap.
enum OverlapType
{
   OT_NONE     = 0,                    // all contacts and overlaps NOT treated as intersections
   OT_INTERNAL = (1 << 0),             // internal (between hole and outer part) contacts treated as intersections (overlaps don't)
   OT_EXTERNAL = (1 << 1),             // external (between outer parts) contacts treated as intersections (overlaps don't)
   OT_FULL = OT_INTERNAL | OT_EXTERNAL // all overlaps and contacts treated as intersections
};

} // End of 'verification' namespace
} // End of 'cg' namespace
