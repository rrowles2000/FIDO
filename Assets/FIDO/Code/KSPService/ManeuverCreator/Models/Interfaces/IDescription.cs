namespace Fido.sources.Models
{
    public interface IDescription
    {
        string description { set; get; }
        string GetDescription();
        void SetDescription(string description);
        

    }
}