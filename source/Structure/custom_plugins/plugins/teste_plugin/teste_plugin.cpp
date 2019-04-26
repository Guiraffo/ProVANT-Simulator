

#include <teste_plugin.h>

namespace gazebo
{
	
	

	// constructor
	testePlugin::testePlugin()
	{ 

	}

	// destructor
	testePlugin::~testePlugin()
	{	
		
	}

	// initial setup
	void testePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		std::cout << "Hello World" << std::endl;
		
		world = _model->GetWorld(); // get World's pointer
		updateTimer.Load(world, _sdf);
	  	updateConnection = updateTimer.Connect(boost::bind(&testePlugin::Update, this)); //Codigo de update tirado da internet ...so aceita
	}
	
	// reset
	void testePlugin::Reset()
	{
		
	}

	// for each ste time
	void testePlugin::Update()
	{
		std::cout << "Hello World2" << std::endl;
	}
	
	GZ_REGISTER_MODEL_PLUGIN(testePlugin)
}
