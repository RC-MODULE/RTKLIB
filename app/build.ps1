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

Try {
    $StartTime = Get-Date
    $prior_location = Get-Location
    [Console]::OutputEncoding = [System.Text.Encoding]::GetEncoding("cp866")
    $script_location = $MyInvocation.MyCommand.Definition
    $msBuild2015 = "${env:ProgramFiles(x86)}\MSBuild\14.0\bin\msbuild.exe"

    $msvc_projects = @("convbin", "pos2kml", "rnx2rtkp")
    ForEach ($prj in $msvc_projects) {
        build_msvc($prj)
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
}
Catch{
    cd $prior_location
}