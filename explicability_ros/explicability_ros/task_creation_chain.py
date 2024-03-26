from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate
from langchain.schema.language_model import BaseLanguageModel


class TaskCreationChain(LLMChain):
    """Chain generating tasks."""

    @classmethod
    def from_llm(cls, llm: BaseLanguageModel, verbose: bool = True) -> LLMChain:
        """Get the response parser."""
        task_creation_template = (
            "You are an explainability AI tool for ROS 2 mobile robots. "
            "Your goal is to detect possible cyberattacks. "
            "You have to detect anomalous behaviour within the robot's data. "
            "You have to interpret the robot's data paying attention to the publication rates of cmd_vel, that is between is 8.0Hz and 23.0Hz.\n\n"

            "Relevant data:\n"
            "{logs}\n\n"

            "USER: "
            "Given the context information and not prior knowledge, answer the query: {question}\n"
            "ASSISTANT:"
        )
        prompt = PromptTemplate(
            template=task_creation_template,
            input_variables=[
                "question",
                "logs",
            ],
        )
        return cls(prompt=prompt, llm=llm, verbose=verbose)
