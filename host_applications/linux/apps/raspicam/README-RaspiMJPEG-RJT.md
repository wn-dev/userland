RJTidey Fork
Mar 5th 2015
Raspimjpeg support for separate settings for time lapse
Raspimjpeg copy cam.jpg as thumbnails for captures

Mar 6th 2015
Raspimjpeg loading settings moved to a separate function and
made more tolerant of syntax / spacing. Config file can point to
a separate user config file which can overwrite values.

March 7th 2015
Config files are reloaded during startall operations so if stopped and started
they take effect
Command Pipe cleared out during start up.
Command processing moved to separate routine

March 8th 2015
Change thumbnail naming to append [vit]Number.th.jpg
THis is used by preview to tie thumbnail to source irrespective of base naming

March 9th 2015
Naming scheme switched to using different letters for date fields and image counts
which may be put in any order
Add in annotation to same naming scheme

March 10th 2015
Removed command line switches to set image and video indexes
These are now automatically set within raspimjpeg by a directory scan

March 11th 2015
Fix for motion triggered time lapse

March 18th
Naming rules can now be used to create sub-folders under the base media path
Thumbnails are stored in the root media but have a flattened pathname to the main capture incuded
