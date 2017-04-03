
require 'rake'

require 'path'

class BenchmarkTasks

  include Rake::DSL

  def initialize( builds, opts = {} )
    @builds = builds

    @dataset_dir = ROOT_DIR.parent.join('datasets/')

    @binary = "LSD_GUI"

    @bench_directory = ROOT_DIR.join("bench_results")


    define_tasks
  end

  def define_tasks
    @builds.each { |build|
      build.in_namespace do
        namespace :bench do
          define_benchmark_tasks( build )
        end
      end
    }
  end

  def define_benchmark_tasks( build )
    ## Nasty, ugly, nasty

      { room: "LSD_room",
        machine: "LSD_machine",
        office: "Zed_office" }.each_pair {|name,dataset_path|

      desc "Run the \"name\" test set with \"#{build}\""
      task name do
        room_dir = @bench_directory.join(name.to_s)
        mkdir_p room_dir unless FileTest.directory? room_dir

        data_dir = @dataset_dir.join(dataset_path)

        chdir room_dir do
          sh "#{build.build_dir}/#{@binary} -c #{data_dir}/cameraCalibration.cfg -f #{data_dir}/images/"
        end
      end
    }
    end


end
