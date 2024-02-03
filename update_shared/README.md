### Shared files autoexport

Automatically export to the workspace's share path yaml and launch file upon saving, without recompiling the sources all the times. To do so:

## Install `update_shared.sh`

In the directory of `update_shared`, run:

* `sudo make install` to install the script to `/usr/local/bin`
* `sudo make uninstall` to remove the script

This script will get the package from the directory it is executed from (can be any subfolder), and runs `colcon build` for only that package and setting the cmake option `${shared_only}`` to true.

## Configure your CMakeLists.txt

Group all the source targets and everything you don't want to build when exporting the shared files under an if statement, like so:
```
...

install (DIRECTORY
	launch # eg.
	config
	# ...
	DESTINATION share/${PROJECT_NAME}
)


if (NOT ${share_only})
	add_executable ...
	...
	install (TARGET ...
endif()
```

## Configure your editor

Configure your editor to run `update_shared.sh` upon saving a shared file. 
### `sublime-text`

Here is a possible configuration for `sublime-text` editor.

1. **Create a Custom Build system**
	- From Sublime Text, go to `Tools > Build System > New Build System...`.
	- Set up the build system like so:
	```
	{
		"shell_cmd": "update_shared.sh $file",
	  	"selector": "source.yaml, source.launch",
		"working_dir": "$file_path",
		"file_patterns": ["*.yaml","*.launch"]
	}
	```
	In this example it is set for `.yaml` and `.launch` files only, but it can be any extension.
	* Save this file with a name like ros2_share.sublime-build, in the location Sublime suggests.
2. **Enable automatic building on save for specific file types**:
	- Install `SublimeOnSaveBuild`, using Package Control
		- Open the command Palette with `Ctrl+Shift+P`
		- Type "Install Package", hit enter, search for "SublimeOnSaveBuild" and install it.
	- By default `SublimeOnSaveBuild` triggers for any file type. To restrict it to those you require:
		- Go to `Preferences > Package Settings > SublimeOnSaveBuild > Settings - User`.
		- Add the following configuration, adapting it to your preferred file types:
		```
		{
			"filename_filter": "\\.(yaml|launch)$",
		}
		```