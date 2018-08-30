function build_msvc {
Param
    (
        [Parameter(Position=0)] # Positional parameter
        [string[]]$project_name
    )

    Try{
        $cur_dir = "$project_name" + "\msvc"
        $solution_path = "$cur_dir"+"\"+"$project_name"+".sln"

        &$msBuild2015 $solution_path /t:Rebuild /p:Configuration=Release /p:Platform="x86" /m
        If($LastExitCode -gt 0){
            throw
        }
        
        &$msBuild2015 $solution_path /t:Rebuild /p:Configuration=Release /p:Platform="x64" /m
        If($LastExitCode -gt 0){
            throw
        }

        copy $cur_dir\Release\$project_name.exe ..\bin\$project_name.exe
        copy $cur_dir\x64\Release\$project_name.exe ..\bin\"$project_name"_win64.exe 

    }
    Catch{
        echo "Error installing $project_name"
	    throw
    }
}

function build_borland {
Param
    (
        [Parameter(Position=0)]
        [string[]]$project_name,

        [Parameter(Position=1)]
        [string[]]$project_architecture
    )

    Try{
        $prior = Get-Location
        $cur_dir = "$project_name"
        cd $cur_dir
        $solution_path = "$project_name"+".cbproj"
        Remove-Item -Force -Recurse "Release_Build"

        &$msBuild2015 $solution_path /t:Build /p:Configuration=Release /p:Platform="$project_architecture" /m
        If($LastExitCode -gt 0){
            throw
        }

        copy Release_Build\$project_name.exe ..\..\bin\$project_name.exe
        cd $prior
    }
    Catch{
        echo "Error installing $project_name"
	    throw
    }
}

function build_rnx2rtkp_win64 {
    Try{
        $prior = Get-Location
        $cur_dir = "rnx2rtkp" + "\bcc_win64\"
        cd $cur_dir
        $solution_path = "_rnx2rtkp_win64.cbproj"
        Remove-Item -Force -Recurse "Release_Build"

        &$msBuild2015 $solution_path /t:Build /p:Configuration=Release /p:Platform="Win64" /m
        If($LastExitCode -gt 0){
            throw
        }

        $src = "Release_Build\_rnx2rtkp_win64.exe"
        copy $src ..\..\..\bin\rnx2rtkp_win64.exe
        cd $prior
    }
    Catch{
        echo "Error installing $project_name"
	    throw
    }
}


function build_borland_console {
Param
    (
        [Parameter(Position=0)]
        [string[]]$project_name,

        [Parameter(Position=1)]
        [string[]]$project_architecture
    )

    Try{
        $prior = Get-Location
        $cur_dir = "$project_name" + "\bcc\"
        cd $cur_dir
        $solution_path = "_" + "$project_name"+".cbproj"
        Remove-Item -Force -Recurse "Release_Build"

        &$msBuild2015 $solution_path /t:Build /p:Configuration=Release /p:Platform="$project_architecture" /m
        If($LastExitCode -gt 0){
            throw
        }

        $src = "Release_Build\_" + "$project_name.exe"
        copy $src ..\..\..\bin\$project_name.exe
        cd $prior
    }
    Catch{
        echo "Error installing $project_name"
	    throw
    }
}

Try {
    $StartTime = Get-Date
    $prior_location = Get-Location
    ipconfig|out-null;[Console]::OutputEncoding = [System.Text.Encoding]::GetEncoding("cp866")
    $script_location = $MyInvocation.MyCommand.Definition
    $msBuild2015 = "${env:ProgramFiles(x86)}\MSBuild\14.0\bin\msbuild.exe"

	build_msvc rtkrcv
    build_rnx2rtkp_win64

	$boarland_console = @("convbin", "pos2kml", "rnx2rtkp")
    ForEach ($prj in $boarland_console) {
        build_borland_console $prj "Win32"
    }

    $boarland_projects = @("rtkconv", "rtkget", "rtklaunch", "rtknavi", "rtknavi_mkl", "rtkplot", "rtkpost", "rtkpost_mkl", "srctblbrows", "strsvr")
    ForEach ($prj in $boarland_projects) {
        build_borland $prj "Win32"
    }

    $boarland_projects = @("rtknavi_win64", "rtkpost_win64")
    ForEach ($prj in $boarland_projects) {
        build_borland $prj "Win64"
    }
    
    $dst = ((Get-Date) - $StartTime).TotalSeconds
    "Total build time: " + $dst + " seconds"
	./zip-up.ps1
}
Catch{
    cd $prior_location
}